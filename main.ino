#include <Wire.h>
#include <PID_v1.h>
#include <SPI.h>
#include <Matrix.h>
#include <math.h>
#include <I2Cdev.h>
#include <MPU6050.h>

#define ENGINES
//#define COMPUTE_PID
//#define COMPUTE_KF
#define ENCODERS
//#define IMU
//#define ULTRASONS

//#define DEBUG

//Define if communication between Raspberry Pi and Arduino is by SPI or I2C
//#define COMMUNICATION_I2C
#define COMMUNICATION_SPI


#define CONTROLPOSITIONPOINTS 1
#define CONTROLCARTESIANSPEED 2
#define CONTROLROBOTSPEED 3
#define CONTROLPOSITIONNEXTPOINT 4


#define CHANGETYPEOFCONTROL 1
#define RECEIVEORIENTATION 2
#define ENABLEENGINES 3
#define DISABLEENGINES 4
#define SENDDATA 5
#define RESETREF 6
#define OFFULTRASON 7
#define ULTRASONSOFT 8
#define ULTRASONHARD 9

#define RISE 0
#define FALL 1

#define SOFTREACTION 0
#define HARDREACTION 1

//#define SLOPING_FLOOR

// Kalman filter states and measurements
#ifdef SLOPING_FLOOR
  #define NSTATES 15
  #define NSEEN 9  
#endif
#ifndef SLOPING_FLOOR
  #ifdef ENCODERS
    #ifdef IMU
      #define NSTATES 8
      #define NSEEN 6
    #endif
    #ifndef IMU
      #define NSTATES 6
      #define NSEEN 3
    #endif
  #endif
  #ifndef ENCODERS
    #ifdef IMU
      #define NSTATES 8
      #define NSEEN 3
    #endif
    #ifndef IMU
      #define NSTATES 6
      #define NSEEN 3
    #endif
  #endif
#endif

#define readOnePositionSpi(result)\
  do{\
    result = 0;\
    result |= ((int)spiBuffer[spiPos++] & 0x00FF);\
    result <<= 8;\
    result |= ((int)spiBuffer[spiPos++] & 0x00FF);\
  }while(0);
  
#define readOnePositionI2C(result)\
  do{\
    result = 0;\
    result |= ((int)Wire.read() & 0x00FF);\
    result <<= 8;\
    result |= ((int)Wire.read() & 0x00FF);\
  }while(0);
  
// Debug variables
long int timeDebugT1 = 0;
long int timeDebugT2 = 0;
long int timeDebugT3 = 0;

//Interrupts flags
boolean flagIntT1 = 0;
boolean flagIntT2 = 0;
boolean flagIntT3 = 0;

// Pins numbers
const unsigned char pwmMPin[] = {6, 7, 8 };  
const int enM1Pin = 33;   
const int enM2Pin = 35;
const int enM3Pin = 37;
const int ultrasonAddressPin[] = {44, 42, 40};
const int ultrasonTrigPin = 38;
const int ultrasonEchoPin = 18; //interrupt 5
const int encoder1APin = 2;
const int encoder2APin = 3;
const int encoder3APin = 19;
const int encoder1BPin = 32;
const int encoder2BPin = 34;
const int encoder3BPin = 36;

//PWM mode
const byte modePWM = 0x08; // Prescale of 1

//Robot constants
const float RRobot = 150; //Robot Radius in mm
const float RFreeRobot = 120; //Radius of free wheels support in mm
const float RWheel = 34.5; // wheel radius in mm
const int tourPulses = 1024; // Encoder resolution 1024

// Sample times
const float AccSampleTime = 0.01;
const float PIDSampleTime = 0.01; //in s
const float KFSampleTime = PIDSampleTime; //in s
const float encoderSampleFreq = 2/PIDSampleTime; //in Hertz
const float UltrasonSampleTime = 0.06;

// Variable used in SPI communication
volatile boolean processIt;

// Ultrason Parameters
const int nbUltrasons = 6;
const float reactionDistance = 6; //cm
const float hardLimit = 3;
const int ultrasonAngle[nbUltrasons]={PI, 2*PI/3, PI/3, 0, -PI/3, -2*PI/3};
boolean sonicProtectionStatus = HARDREACTION;
volatile boolean flagUltrason = RISE;
volatile unsigned long echoTime = 0;
volatile byte distanceObstacles[nbUltrasons] = {100,100,100,100,100,100}; // in cm
volatile byte indexUltrason = 0;

// Encoder Mesures
volatile double deltaXEnc = 0; // robot deplacement in x
volatile double deltaYEnc = 0;
volatile double deltaTetaEnc = 0; 
volatile double XEnc = 0; // Coordinate x of the robot
volatile double YEnc = 0;
volatile double tetaEnc = 0; //robot orientation
volatile double VitXEnc=0;
volatile double VitYEnc=0;
volatile double VitTetaEnc=0;

// Engine enable boolean variable
boolean enEngine = LOW;

// Positions Array variables
const int maxNumberPositions = 50;
const int nbCoord = 3;
volatile int indexPosition = 0;
int nbPoints;
int arrayPositions[maxNumberPositions][nbCoord];

// SPI Parameters
#ifdef ULTRASONS
int nbBytesSpi = 32;
#endif
#ifndef ULTRASONS
int nbBytesSpi = 26;
#endif
char spiBuffer [100];
volatile byte spiPos = 0;
char bufferSendData [100];
volatile boolean flagSendData = 0;

#ifdef SLOPING_FLOOR
  //Measurements vector - {XEnc, YEnc, tetaEnc, VitTetaXG, VitTetaYG, VitTetaZG, AccX, AccY, AccZ}
  REAL measurements[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
#endif
#ifndef SLOPING_FLOOR
  #ifdef ENCODERS
    #ifdef IMU
      //Measurements vector - {XEnc, YEnc, tetaEnc, AccX, AccY, VitTetaZG}
      REAL measurements[] = {0, 0, 0, 0, 0, 0};
    #endif
    #ifndef IMU
      //Measurements vector - {XEnc, YEnc, tetaEnc}
      REAL measurements[] = {0, 0, 0};
    #endif
  #endif
#endif

#ifndef ENCODERS
//Measurements vector - {AccX, AccY, VitTetaZG}
REAL measurements[] = {0, 0, 0};
#endif


// Kalman Parameters to Q matrix - Look at article "Kalman Filter Implementation on an Accelerometer sensor data for three state estimation of a dynamic system" as reference
const float q = 0.01;
float QX = q*pow(KFSampleTime,5)/20;
float QY = q*pow(KFSampleTime,5)/20;
float QZ = q*pow(KFSampleTime,5)/20;
float QTeta = q*pow(KFSampleTime,5)/20;
float QVitTetaX = q*pow(KFSampleTime,3)/3;
float QVitTetaY = q*pow(KFSampleTime,3)/3;
float QVitTeta = q*pow(KFSampleTime,3)/3;
float QVitX = q*pow(KFSampleTime,3)/3;
float QVitY = q*pow(KFSampleTime,3)/3;
float QVitZ = q*pow(KFSampleTime,3)/3;
float QAccX = q*KFSampleTime/1;
float QAccY = q*KFSampleTime/1;
float QAccZ = q*KFSampleTime/1;
float Qpv = q*pow(KFSampleTime,4)/8;
float Qpa = q*pow(KFSampleTime,3)/6;
float Qva = q*pow(KFSampleTime,2)/2;
#ifdef SLOPING_FLOOR 
  REAL processCovar[] = {  QX,    0,     0,     0,     0,       0,       Qpv,      0,       0,      0,         0,         0,         Qpa,    0,       0,      
                           0,     QY,    0,     0,     0,       0,       0,        Qpv,     0,      0,         0,         0,         0,      Qpa,     0,  
                           0,     0,     QZ,    0,     0,       0,       0,        0,       Qpv,    0,         0,         0,         0,      0,       Qpa,   
                           0,     0,     0,     QTetaX,0,       0,       0,        0,       0,      Qpv,       0,         0,         0,      0,       0,   
                           0,     0,     0,     0,     QTetaY,  0,       0,        0,       0,      0,         Qpv,       0,         0,      0,       0,    
                           0,     0,     0,     0,     0,       QTetaZ,  0,        0,       0,      0,         0,         Qpv,       0,      0,       0,   
                           Qpv,   0,     0,     0,     0,       0,       QVitX,    0,       0,      0,         0,         0,         Qva,    0,       0,  
                           0,     Qpv,   0,     0,     0,       0,       0,        QVitY,   0,      0,         0,         0,         0,      Qva      0,  
                           0,     0,     Qpv,   0,     0,       0,       0,        0,       QVitZ,  0,         0,         0,         0,      0,       Qva,   
                           0,     0,     0,     Qpv,   0,       0,       0,        0,       0,      QVitTetaX, 0,         0,         0,      0,       0,   
                           0,     0,     0,     0,     Qpv,     0,       0,        0,       0,      0,         QVitTetaY, 0,         0,      0,       0,   
                           0,     0,     0,     0,     0,       Qpv,     0,        0,       0,      0,         0,         QVitTetaZ, 0,      0,       0,   
                           Qpa,   0,     0,     0,     0,       0,       Qva,      0,       0,      0,         0,         0,         QAccX,  0,       0,
                           0,     Qpa,   0,     0,     0,       0,       0,        Qva,     0,      0,         0,         0,         0,      QAccY,   0,
                           0,     0,     Qpa,   0,     0,       0,       0,        0,       Qva,    0,         0,         0,         0,      0,       QAccZ};
#endif
#ifndef SLOPING_FLOOR
  #ifdef IMU
    REAL processCovar[] = {QX,     0,     0,     Qpv,     0,       0,        Qpa,   0,  
                           0,     QY,    0,     0,       Qpv,     0,        0,     Qpa,
                           0,     0,     QTeta, 0,       0,       Qpv,      0,     0, 
                           Qpv,   0,     0,     QVitX,   0,       0,        Qva,   0, 
                           0,     Qpv,   0,     0,       QVitY,   0,        0,     Qva,  
                           0,     0,     Qpv,   0,       0,     QVitTeta,   0,     0,
                           Qpa,   0,     0,     Qva,     0,       0,        QAccX, 0, 
                           0,     Qpa,   0,     0,       Qva,     0,        0,     QAccY};
  #endif
  #ifndef IMU
    REAL processCovar[] = {QX,     0,     0,     Qpv,     0,       0,         
                           0,     QY,    0,     0,       Qpv,     0,        
                           0,     0,     QTeta, 0,       0,       Qpv,      
                           Qpv,   0,     0,     QVitX,   0,       0,        
                           0,     Qpv,   0,     0,       QVitY,   0,        
                           0,     0,     Qpv,   0,       0,     QVitTeta};

  #endif
#endif

// Kalman Parameters to R matrix                        
float varXEnc2 = pow(25, 2); // mm
float varYEnc2 = pow(25, 2);
float varTetaEnc2 = pow(0.0205,2);
float varAcc2 = pow(1.4143,2); // based on http://www.floconcept.fr/realisations/quadwiicopter/fichiers/CentraleIntertielle.pde
float varGyr2 = pow(0.32,2); //rad/s

#ifdef SLOPING_FLOOR
  REAL measCovar[] = {varXEnc2,   0,             0,             0,               0,          0,         0,        0,        0,
                          0,         varYEnc2,   0,             0,               0,          0,         0,        0,        0,
                          0,         0,          varTetaEnc2,   0,               0,          0,         0,        0,        0,
                          0,         0,          0,             varGyr2,         0,          0,         0,        0,        0,
                          0,         0,          0,             0,               varGyr2,    0,         0,        0,        0,
                          0,         0,          0,             0,               0,          varGyr2,   0,        0,        0,
                          0,         0,          0,             0,               0,          0,         varAcc2,  0,        0,
                          0,         0,          0,             0,               0,          0,         0,        varAcc2,  0,
                          0,         0,          0,             0,               0,          0,         0,        0,        varAcc2};   
#endif
#ifndef SLOPING_FLOOR
  #ifdef ENCODERS
    #ifdef IMU
      REAL measCovar[] = {varXEnc2,   0,          0,             0,               0,          0,
                          0,         varYEnc2,   0,             0,               0,          0,
                          0,         0,          varTetaEnc2,   0,               0,          0,
                          0,         0,          0,             varGyr2, 0,          0,
                          0,         0,          0,             0,               varAcc2,   0,
                          0,         0,          0,             0,               0,          varAcc2};                    
    #endif
    #ifndef IMU
      REAL measCovar[] = {varXEnc2,   0,          0,                          
                     0,         varYEnc2,   0,                           
                     0,         0,          varTetaEnc2};              
    #endif
  #endif

  #ifndef ENCODERS
    #ifdef IMU
      REAL measCovar[] = { varGyr2,         0,          0,
                           0,               varAcc2,    0,
                           0,               0,          varAcc2};                    
    #endif
    #ifndef IMU
      float varN = 0;
      REAL measCovar[] = { varN,         0,         0,
                           0,         varN,         0,
                           0,         0,         varN};                    
    #endif
  #endif
#endif

// Matrix P
#ifdef SLOPING_FLOOR
REAL uncertainty[] = {    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
#endif
#ifndef SLOPING_FLOOR
#ifdef IMU
REAL uncertainty[] = {0, 0, 0, 0, 0, 0, 0, 0, 
                          0, 0, 0, 0, 0, 0, 0, 0, 
                          0, 0, 0, 0, 0, 0, 0, 0, 
                          0, 0, 0, 0, 0, 0, 0, 0, 
                          0, 0, 0, 0, 0, 0, 0, 0, 
                          0, 0, 0, 0, 0, 0, 0, 0, 
                          0, 0, 0, 0, 0, 0, 0, 0, 
                          0, 0, 0, 0, 0, 0, 0, 0}; 
#endif
#ifndef IMU
REAL uncertainty[] = {    0, 0, 0, 0, 0, 0, 
                          0, 0, 0, 0, 0, 0, 
                          0, 0, 0, 0, 0, 0, 
                          0, 0, 0, 0, 0, 0, 
                          0, 0, 0, 0, 0, 0, 
                          0, 0, 0, 0, 0, 0}; 

#endif
#endif
                          
//Robot State vector to Kalman Filter
#ifdef SLOPING_FLOOR
REAL state[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //{X, Y, Z, tetaX, tetaY, tetaZ, VitX, VitY, VitZ, VitTetaX, VitTetaY, VitTetaZ, AccX, AccY, AccZ}
#endif
#ifndef SLOPING_FLOOR
#ifdef IMU
REAL state[] = {0, 0, 0, 0, 0, 0, 0, 0}; //{X, Y, teta, VitX, VitY, VitTeta, AccX, AccY}
#endif
#ifndef IMU
REAL state[] = {0, 0, 0, 0, 0, 0}; //{X, Y, teta, VitX, VitY, VitTeta}
#endif
#endif

//Parameters to position control
float errorPosition = 0.01; //robot distance of target position 
volatile long int counterEncoder1 = 0;
volatile long int counterEncoder2 = 0;
volatile long int counterEncoder3 = 0;
volatile long int counterEncoder1Old = 0;
volatile long int counterEncoder2Old = 0;
volatile long int counterEncoder3Old = 0;

//contants for the position control PID
double const KpPos=100, KiPos=0.02, KdPos=50;

//contants for the speed control PID
double const KpVit=200, KiVit=1, KdVit=0;

//type of control of the robot that is momentanely used
//possibly control in positon (0)
//control in cartesian speeds (1)
//control in relative to the robot speeds (2)
volatile unsigned short typeOfControl = 0;

//type of command
//type of control: 0
//enable engines: 1
//disable engines: 2
//enable interrupts: 3
volatile unsigned short typeOfCommand;

// Robot speed desired in own reference
volatile double VitXRobotR=0, LastVitXRobotR=0;
volatile double VitYRobotR=0, LastVitYRobotR=0;
volatile double VitTetaRobotR=0, LastVitTetaRobotR=0;

// Coordinates of robot
double X = 0; // Coordinate x of the robot
double Y = 0;
double Z = 0;
double teta = 0; //robot orientation
double tetaX = 0;
double tetaY = 0;

// Robot speed in external reference (map)
double VitX=0, VitXR=0, LastVitXR=0, XR=0;
double VitY=0, VitYR=0, LastVitYR=0, YR=0;
double VitTeta=0, VitTetaR=0, LastVitTetaR=0, TetaR=0;
double VitTetaX=0, VitTetaY=0;
double w1=0, E1=0, w1R=0;
double w2=0, E2=0, w2R=0;
double w3=0, E3=0, w3R=0;

// Speed limits
const double maxVitX = 500;
const double maxVitY = 500;
const double maxVit = 500;
const double maxVitTeta = 4.1;

// Parameters for function coneRefreshSpeed
const float percLimVitTetaR = 0.2;
const float limVitTetaR = percLimVitTetaR*maxVitTeta;

// Parameters for function coneRefreshAcceleration
const float maxAccelTranslation = 1000/4;
const float maxAccelTeta = 8.2/4;
const float percLimAccelTetaR = 0.2;
const float limAccelTetaR = percLimAccelTetaR*maxAccelTeta;
  
// IMU data
MPU6050 accelgyro(0x69); // <-- use for AD0 high
int16_t AccX = 0; 
int16_t AccY = 0; 
int16_t AccZ = 0;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t VitTetaZG = 0; 
int16_t VitTetaYG = 0; 
int16_t VitTetaXG = 0; 

//Specify the links and initial tuning parameters for the PID for each position parameter
PID PIDPosX(&X, &VitXR, &XR, KpPos, KiPos, KdPos, DIRECT);
PID PIDPosY(&Y, &VitYR, &YR, KpPos, KiPos, KdPos, DIRECT);
PID PIDPosTeta(&teta, &VitTetaR, &TetaR, KpPos, KiPos, KdPos, DIRECT);

//Specify the links and initial tuning parameters for the PID for each engine tension
PID PIDVit1(&w1, &E1, &w1R, KpVit, KiVit, KdVit, DIRECT);
PID PIDVit2(&w2, &E2, &w2R, KpVit, KiVit, KdVit, DIRECT);
PID PIDVit3(&w3, &E3, &w3R, KpVit, KiVit, KdVit, DIRECT);

//Transform references absolute speeds in references wheel angular speeds, depending in teta
void vitR2wR(){
  float rac3 =1.732;
  float cosTeta = cos(teta);
  float sinTeta = sin(teta);
  w1R=0.5*(2*RRobot*VitTetaR-(rac3*VitYR+VitXR)*cosTeta+(-VitYR+rac3*VitXR)*sinTeta)/RWheel;
  w2R=0.5*(2*RRobot*VitTetaR+(rac3*VitYR-VitXR)*cosTeta+(-VitYR-rac3*VitXR)*sinTeta)/RWheel;
  w3R=(RRobot*VitTetaR+VitXR*cosTeta+VitYR*sinTeta)/RWheel;
}

//Transform references robot-referenced speeds in references wheel angular speeds, depending in teta
void vitRobotR2wR(){
  float rac3 = 1.732;
  w1R=(RRobot*VitTetaRobotR+0.5*(-rac3*VitYRobotR-VitXRobotR))/RWheel;
  w2R=(RRobot*VitTetaRobotR+0.5*(rac3*VitYRobotR-VitXRobotR))/RWheel;
  w3R=(RRobot*VitTetaRobotR+VitXRobotR)/RWheel;
}

//Transform wheel angular speeds in cartesian speeds, depending in teta
//not used
void w2vit(){
  float rac3 =1.732;
  float cosTeta = cos(teta);
  float sinTeta = sin(teta);
  VitX=((rac3*w3-rac3*w1)*sinTeta+(2*w2-w3-w1)*cosTeta)*RWheel/3;
  VitY=-((rac3*w3-rac3*w1)*cosTeta+(-2*w2+w3+w1)*sinTeta)*RWheel/3;
  VitTeta=(w1+w2+w3)*RWheel/3/RRobot;
}

// Transform cartesian speeds in robot reference speeds
//not used
void cart2robot(){
  float cosTeta = cos(teta);
  float sinTeta = sin(teta);
  VitXRobotR = sinTeta*VitXR + cosTeta*VitYR;
  VitYRobotR = - cosTeta*VitXR - sinTeta*VitYR;
  VitTetaRobotR = VitTetaR;
}

// Transform robot reference speeds in absolute-referenced speeds
//not used
void robot2cart(){
  float cosTeta = cos(teta);
  float sinTeta = sin(teta);
  VitXR = - sinTeta*VitXRobotR + cosTeta*VitYRobotR;
  VitYR = - cosTeta*VitXRobotR + sinTeta*VitYRobotR;
  VitTetaR = VitTetaRobotR;
}

//Calculate robot distance to the target position
double distanceTargetPosition(){
  return sqrt((XR-X)*(XR-X)+(YR-Y)*(YR-Y));
}

// Compute all PID's
// Verify the type of control requested
void computeAllPID(){
  
  //unsigned long timePIDComputeAll = micros();
  
  if(typeOfControl==CONTROLPOSITIONNEXTPOINT||typeOfControl==CONTROLPOSITIONPOINTS){
    if(indexPosition<nbPoints-1){  
      if(distanceTargetPosition()<errorPosition){
        indexPosition++;
        XR = arrayPositions[indexPosition][0];
        YR = arrayPositions[indexPosition][1];
      }
    }
    PIDPosX.Compute();
    PIDPosY.Compute();
    PIDPosTeta.Compute();    
  }
  
  coneRefreshSpeed();

  //change speeds to have a good acceleration (not too high)
  coneRefreshAcceleration();
  
  if(typeOfControl==CONTROLROBOTSPEED)
  { 
    VitXRobotR = LastVitXRobotR;
    VitYRobotR = LastVitYRobotR;
    VitTetaRobotR = LastVitTetaRobotR;
    vitRobotR2wR();
  }
  else if(typeOfControl==CONTROLCARTESIANSPEED)
  {
    VitXR = LastVitXR;
    VitYR = LastVitYR;
    VitTetaR = LastVitTetaR;
    vitR2wR();  
  } 
  
  PIDVit1.Compute();
  PIDVit2.Compute();
  PIDVit3.Compute();
  
  #ifdef DEBUG
//  Serial.print("Time os computing all PIDs : ");
//  timePIDComputeAll-=micros();
//  Serial.println(timePIDComputeAll);
//  
  #endif
}

void turnOnAllPID(){
  PIDPosX.SetMode(AUTOMATIC);
  PIDPosY.SetMode(AUTOMATIC);
  PIDPosTeta.SetMode(AUTOMATIC);
  PIDVit1.SetMode(AUTOMATIC);
  PIDVit2.SetMode(AUTOMATIC);
  PIDVit3.SetMode(AUTOMATIC);
}

void enableEngines(){
  enEngine = HIGH;
  digitalWrite(enM3Pin, enEngine); 
  digitalWrite(enM2Pin, enEngine); 
  digitalWrite(enM1Pin, enEngine);  
}

void disableEngines(){
  enEngine = LOW;
  digitalWrite(enM3Pin, enEngine); 
  digitalWrite(enM2Pin, enEngine); 
  digitalWrite(enM1Pin, enEngine);  
}

void setEngines(){
  #ifdef DEBUG
//  Serial.println("seteng");
//  Serial.println(enEngine);
  #endif
  unsigned int pwr1, pwr2, pwr3;
  pwr1 = round(127.5 + E1/12*127.5);
  pwr2 = round(127.5 + E2/12*127.5);
  pwr3 = round(127.5 + E3/12*127.5);
  #ifdef DEBUG
//  Serial.println(pwr1);
//  Serial.println(pwr2);
//  Serial.println(pwr3);
  #endif
  analogWrite(pwmMPin[0], pwr1);  
  analogWrite(pwmMPin[1], pwr2);
  analogWrite(pwmMPin[2], pwr3);
}

void disableTimerInterrupts(){
  #ifdef ULTRASONS
  TIMSK1 &= !(1 << OCIE1A);
  #endif
  #ifdef ENCODERS
  TIMSK2 &= !(1 << OCIE2A);  // disable timer encoder compare interrupt 
  //counterEncoder1 = 0;  // reset counters
  //counterEncoder2 = 0;
  //counterEncoder3 = 0;
  #endif
  TIMSK3 &= !(1 << OCIE3A);  // disable timer PID compare interrupt   
}

void enableTimerInterrupts(){
  #ifdef ULTRASONS
    TIMSK1 |= (1 << OCIE1A);
  #endif
  #ifdef ENCODERS
    TIMSK2 |= (1 << OCIE2A);  // enable timer encoder compare interrupt 
  #endif
    TIMSK3 |= (1 << OCIE3A);  // enable timer PID compare interrupt   
}

// Called in the beginning of the execution of the code
void setup()
{  
  
  noInterrupts();           // disable all interrupts
  #ifdef DEBUG
  Serial.begin(115200);
  #endif
  #ifdef COMMUNICATION_I2C
  Wire.begin(0x2E);                // join i2c bus with address #2
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
  #endif
  
  #ifdef COMMUNICATION_SPI  
  // SPI initialization in slave mode 
  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);  
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  // turn on interrupts
  SPCR |= _BV(SPIE); 
  SPI.setDataMode(SPI_MODE3); 
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.attachInterrupt(); 
  processIt = false;
  #endif
 
  #ifdef ULTRASONS
    pinMode(ultrasonTrigPin, OUTPUT);
    pinMode(ultrasonEchoPin, INPUT);
    pinMode(ultrasonAddressPin[0], OUTPUT);
    pinMode(ultrasonAddressPin[1], OUTPUT);
    pinMode(ultrasonAddressPin[2], OUTPUT);
    digitalWrite(ultrasonTrigPin, LOW);
    // Attach rising ultrasons interrupt
    attachInterrupt(5, ultrasonsInterrupt, RISING);   
    // Initialize timer1 - Ultrasound interrupt 
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A = 62500*UltrasonSampleTime;     // compare match register ultrasoundSampleTime*16MHz/256
    TCCR1B |= (1 << WGM12);   // CTC mode
    TCCR1B |= (1 << CS12);    // 1 prescaler  
  #endif
    
  #ifdef ENGINES
  //set the pins that are going to be used
  pinMode(enM3Pin, OUTPUT);      
  pinMode(enM2Pin, OUTPUT); 
  pinMode(enM1Pin, OUTPUT); 
  digitalWrite(enM3Pin, enEngine); 
  digitalWrite(enM2Pin, enEngine); 
  digitalWrite(enM1Pin, enEngine);
  pinMode(pwmMPin[0], OUTPUT); 
  pinMode(pwmMPin[1], OUTPUT); 
  pinMode(pwmMPin[2], OUTPUT);  
  // PWM Frequency of Pins 6, 7 and 8 in 64kHz
  TCCR4A = TCCR4A & 0b11111000 | modePWM;  
  TCCR4B = TCCR4B & 0b11111000 | modePWM;
  TCCR4C = TCCR4C & 0b11111000 | modePWM;
  setEngines();  // command to maintain Engines stopped
  #endif
  
  #ifdef COMPUTE_PID
  //Sample time setting for all PIDs, for PIDs controlling speed it could be faster, but it has a minimun equals of the Encoders sampling Time
  PIDPosX.SetSampleTime(PIDSampleTime);
  PIDPosY.SetSampleTime(PIDSampleTime);
  PIDPosTeta.SetSampleTime(PIDSampleTime);
  PIDVit1.SetSampleTime(PIDSampleTime);
  PIDVit2.SetSampleTime(PIDSampleTime);
  PIDVit3.SetSampleTime(PIDSampleTime);
  //Setting limits of PID to the physical limits of the robot
  PIDPosX.SetOutputLimits(-maxVitX,maxVitX);
  PIDPosY.SetOutputLimits(-maxVitY,maxVitY);
  PIDPosTeta.SetOutputLimits(-maxVitTeta,maxVitTeta);
  PIDVit1.SetOutputLimits(-12,12);
  PIDVit2.SetOutputLimits(-12,12);
  PIDVit3.SetOutputLimits(-12,12);
  //Turn On all PIDs
  turnOnAllPID(); 
  #endif
  
    // Initialize timer3 - PID interrupt 
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;  
  OCR3A = 62500*PIDSampleTime;     // compare match register 16MHz/1*PIDSampleTime
  TCCR3B |= (1 << WGM32);   // CTC mode
  TCCR3B |= (1 << CS32);    // 256 prescaler 
  
  #ifdef ENCODERS  
  pinMode(encoder1APin, INPUT_PULLUP);      
  pinMode(encoder2APin, INPUT_PULLUP); 
  pinMode(encoder3APin, INPUT_PULLUP);
  pinMode(encoder1BPin, INPUT_PULLUP);      
  pinMode(encoder2BPin, INPUT_PULLUP); 
  pinMode(encoder3BPin, INPUT_PULLUP);  
  // Configure all pins of Port C (digital pins 22...29): 0-Input, 1-Output
  //DDRC = 0b00000000;  // PC1 (36): B signal Encoder 3
                      // PC3 (34): B signal Encoder 2
                      // PC5 (32): B signal Encoder 1  
  
  // Initialize timer2 - Encoder interrupt 
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A = 15625/encoderSampleFreq;     // compare match register 16MHz/1024/encoderSampleFreq
  TCCR2B |= (1 << WGM22);   // CTC mode
  TCCR2B |= (1 << CS20);    // 1 prescaler
  TCCR2B |= (1 << CS21); 
  TCCR2B |= (1 << CS22);
  // Attach encoders interrupts
  attachInterrupt(0, encoder1, RISING);
  attachInterrupt(1, encoder2, RISING);
  attachInterrupt(4, encoder3, RISING);   
  #endif
  
  #ifdef IMU
  // Put arduino as I2C master  
  Wire.begin();                // join i2c bus
  sei();
  accelgyro.initialize();
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);
  accelgyro.setFullScaleGyroRange(1);
  accelgyro.setFullScaleAccelRange(1);
  cli();
 #endif
 
  interrupts();             // enable all interrupts (pin interrupts)
  enableTimerInterrupts();
}

void loop()
{  

#ifdef COMMUNICATION_SPI
  #ifdef DEBUG
  //Serial.println("XUXU");
  #endif
  //In case of not process SPI commands correctly
  if(digitalRead(SS) == HIGH && spiPos!=0)
  {
    if(!flagSendData && ((uint8_t) spiBuffer[spiPos-1]) == 128){
      process_it();  
    }
  spiPos = 0;
  }
#endif

if(flagIntT1)
{
  flagIntT1 = 0;
  #ifdef DEBUG
  //Serial.println("T1");
  //timeDebugT3 = micros();
  #endif
  intT1();
  #ifdef DEBUG
  //Serial.println(micros()-timeDebugT3);
  #endif
}

if(flagIntT2)
{
  flagIntT2 = 0;
  #ifdef DEBUG
  //Serial.println("T2");
  //timeDebugT3 = micros();
  #endif
  intT2();
  #ifdef DEBUG
  //Serial.println(micros()-timeDebugT3);
  #endif
}

if(flagIntT3)
{
  flagIntT3 = 0;
  #ifdef DEBUG
  //Serial.println("T3");
  //timeDebugT3 = micros();
  #endif
  intT3();
  #ifdef DEBUG
  //Serial.println(micros()-timeDebugT3);
  #endif
}

}


//Interrupt Encoder 1 - read encoder 1 and change the value of counterEncoder looking the sense of rotation
void encoder1()
{
  //if((PINC&0x32) == 0x32){  // Tests if Port_A(0) (Signal B of encoder1) is HIGH  
  if(digitalRead(encoder1BPin) == HIGH){
    counterEncoder1--;
  }
  else{
    counterEncoder1++;
  }
}

//Interrupt Encoder 2
void encoder2()
{
  //if((PINC&0x08) == 0x08){   // Tests if Port_A(1) is HIGH
  if(digitalRead(encoder2BPin) == HIGH){
    counterEncoder2--;
  }
  else{
    counterEncoder2++;
  }
}

//Interrupt Encoder 3 - read encoder 3 and change the value of counterEncoder looking the sense of rotation
void encoder3()
{  
  //if((PINC&0x02) == 0x02){  // Tests if Port_A(2) is HIGH
  if(digitalRead(encoder3BPin) == HIGH){
    counterEncoder3--;
  }
  else{
    counterEncoder3++;
  }
}

// Calculate robot distances to obstacles by ultrasons data
void ultrasonsInterrupt()
{
  
  if(flagUltrason==FALL) 
  {
    echoTime = micros()-echoTime;
    #ifdef DEBUG
    //Serial.println("UL");
    Serial.println(round(echoTime/58));
    #endif
    distanceObstacles[indexUltrason] = round(echoTime/58); 
    //EICRA |= (1 << ISC10); //change registers directly
    //EICRA |= (1 << ISC11); 
    attachInterrupt(5, ultrasonsInterrupt, RISING);    
  }
  else 
  {
    echoTime = micros();
    //EICRA &= !(1 << ISC10); 
    //EICRA |= (1 << ISC11);
    attachInterrupt(5, ultrasonsInterrupt, FALLING); 
  }
  flagUltrason = !flagUltrason;
}

/// Refresh robot angular velocity and robot linear speed considering limits to the speed for each wheel
// it uses the principle of the velocity cone showed in 
// Velocity and Acceleration Cones for Kinematic and Dynamic Constraints on Omni-Directional Mobile Robots
// by Jianhua Wu and Robert L. Williams II
void coneRefreshSpeed()
{   
  // Calculate the maximal possible angular velocity from the robot current speed
  float maxVitTetaR = maxVitTeta - maxVitTeta*sqrt(VitXR*VitXR+VitYR*VitYR)/maxVit;
  // If desired angular velocity higher than maximal possible angular velocity
  // change desired angular velocity and refresh speed if necessary
  if(VitTetaR > maxVitTetaR)
  {   
 
    if(VitTetaR > limVitTetaR)
    {
     if(maxVitTetaR>limVitTetaR)
     {
         VitTetaR = maxVitTetaR;
     }
     else
     {
        VitTetaR = limVitTetaR; 
        float Vit = maxVit - VitTetaR*maxVit/maxVitTeta; 
        float ratio = sqrt(VitXR*VitXR + VitYR*VitYR)/Vit;
        VitXR = VitXR/ratio;
        VitYR = VitYR/ratio;
     }
    }
    else
    {
      // If desired angular velocity is too small
      // refresh only linear speed
      float Vit = maxVit - VitTetaR*maxVit/maxVitTeta; 
      float ratio = sqrt(VitXR*VitXR + VitYR*VitYR)/Vit;
      VitXR = VitXR/ratio;
      VitYR = VitYR/ratio;  
    }
  }
}

// Refresh robot angular velocity and robot linear speed considering limits to the acceleration of the robot
// The important changed variables are VitTetaR and/or (VitXR and VitYR)
//it uses the principle of the acceleration cone showed in page 793-796 of
//Velocity and Acceleration Cones for Kinematic and Dynamic Constraints on Omni-Directional Mobile Robots
//by Jianhua Wu and Robert L. Williams II
void coneRefreshAcceleration(){
  //get cartesian speeds of the robot
  w2vit();
  //calculate the desired linear acceleration
  float accelTranslation = sqrt((VitXR-VitX)*(VitXR-VitX)+(VitYR-VitY)*(VitYR-VitY))/PIDSampleTime;
  //calculate the desired angular acceleration
  float accelTeta = (VitTetaR - VitTeta)/PIDSampleTime;
  //check the maximal angular acceleraton for current linear acceleration, respecting
  //maximum torque for each engine
  float maxAccelTetaR = maxAccelTeta - maxAccelTeta*accelTranslation/maxAccelTranslation;
  //if the desired angular acceleration is higher than the maximal possible angular acceleration
  if(accelTeta > maxAccelTetaR)
  {   
    if(accelTeta > limAccelTetaR)
    {
      //if the desired angular acceleration is too high
      //change the angular acceleration
      if(maxAccelTetaR>limAccelTetaR)
      {        
        VitTetaR = PIDSampleTime*maxAccelTetaR+VitTeta;// the same thing as AccelTeta = maxAccelTetaR;
      }
      //if both the angular acceleration and the linear acceleration are high
      //change both the angular acceleration and the linear acceleration
      else
      {        
        VitTetaR = PIDSampleTime*limAccelTetaR+VitTeta; //the same thing as AccelTeta = limAccelTetaR; 
        //change acceleration in x and y respecting the ratio between both
        accelTeta = limAccelTetaR; 
        float Accel = maxAccelTranslation - accelTeta*maxAccelTranslation/maxAccelTeta; 
        float ratio = accelTranslation/Accel;
        //doing the same as AccelTranslation=Accel;
        VitXR = (VitXR-VitX)/ratio + VitX; // the same thing as AccelX = AccelX/ratio;
        VitYR = (VitYR-VitY)/ratio + VitY;// the same thing as AccelY = AccelY/ratio;
      }
    }
    //if linear acceleration is too high
    //change linear acceleration
    else
    {
        //change acceleration in x and y respecting the ratio between both
        float Accel = maxAccelTranslation - accelTeta*maxAccelTranslation/maxAccelTeta; 
        float ratio = accelTranslation/Accel;
        VitXR = (VitXR-VitX)/ratio + VitX;// the same thing as AccelX = AccelX/ratio;
        VitYR = (VitYR-VitY)/ratio + VitY;// the same thing as AccelY = AccelY/ratio;
    }
  }
}

// Refresh robot speed considering obstacles detected by ultrasons 
void ultrasonRefreshSpeed()
{
   for(int i=0; i<nbUltrasons; i++) // For each sensor 
   { 
      float sonic = distanceObstacles[i];
      float sx=cos(ultrasonAngle[i]*PI/180), sy=sin(ultrasonAngle[i]*PI/180); 
      if(sonic <= reactionDistance) // If within reaction range 
      { 
          if(sonicProtectionStatus==HARDREACTION) 
          { 
              float m=VitXR*sx+VitYR*sy; 
              if(m>0) // If actualy going toward the obstacle at speed m 
              { 
                  float k; 
                  if(sonic >= hardLimit) // If outside hard danger range 
                  {
                      k=pow((sonic-hardLimit)/(reactionDistance-hardLimit), 2); 
                  }
                  else // If within hard danger range 
                  {
                      k=0; 
                  }
                  // Cutting offending speed component by (1-k)*100 
                  VitXR -= m*(1-k)*sx; 
                  VitYR -= m*(1-k)*sy; 
              } 
          } 
          else if(sonicProtectionStatus==SOFTREACTION) 
          { 
              VitXR -= sx*(maxVit*reactionDistance/sonic); 
              VitYR -= sy*(maxVit*reactionDistance/sonic); 
          }  
      }  
    }
}

//Function called with frequence 1/UltrasonSampleTime to trigger and read ultrason
ISR(TIMER1_COMPA_vect)          // timer 1 compare interrupt service routine
{
#ifdef DEBUG
//    Serial.println("T1");
//    timeDebugT1 = micros();
#endif
    flagIntT1 = 1;
}

void intT1()
{
    #ifdef DEBUG
    //erial.println(indexUltrason);
    #endif
    cli();
     //PORTL = 0x07&indexUltrason; // change the pins directly
    digitalWrite(ultrasonAddressPin[0], (indexUltrason&0x01)!=0);
    digitalWrite(ultrasonAddressPin[1], (indexUltrason&0x02)!=0);
    digitalWrite(ultrasonAddressPin[2], (indexUltrason&0x04)!=0);
    digitalWrite(ultrasonTrigPin, HIGH);                  // Send a 10uS high to trigger ranging
    //PORTL = PORTL||0b00001000;    
    delayMicroseconds(10);
    //PORTL = PORTL&0b11110111;    
    digitalWrite(ultrasonTrigPin, LOW);
    sei();
    if(indexUltrason==0) ultrasonRefreshSpeed();
    indexUltrason++;    
    indexUltrason%=6; 
    
    #ifdef DEBUG
//    Serial.println(micros()-timeDebugT1);
    #endif   
 
}


//Function called with frequence encoderSampleFreq to calculate the deplacements in X, Y and Teta with 
//the values of the encoder counter
ISR(TIMER2_COMPA_vect)          // timer 2 compare interrupt service routine
{
#ifdef DEBUG
//  timeDebugT2 = micros();
//  Serial.println("T2");
#endif
  flagIntT2 = 1;
  
  #ifdef DEBUG
//  Serial.println(micros()-timeDebugT2);
  #endif
}

void intT2()
{
  long int deltaCounter1 = (counterEncoder1-counterEncoder1Old);
  long int deltaCounter2 = (counterEncoder2-counterEncoder2Old);
  long int deltaCounter3 = (counterEncoder3-counterEncoder3Old);
  
  counterEncoder1Old = counterEncoder1;
  counterEncoder2Old = counterEncoder2;
  counterEncoder3Old = counterEncoder3;
  
  w1=deltaCounter1*2*PI/tourPulses*encoderSampleFreq; //in rad/s
  w2=deltaCounter2*2*PI/tourPulses*encoderSampleFreq;
  w3=deltaCounter3*2*PI/tourPulses*encoderSampleFreq;
  
  float cosTeta = cos(teta);
  float sinTeta = sin(teta);
  
  // The negative sign at the beginning is necessary to obtain the deplacement in robot reference
  //cause the encoders are mirrored from the motors (deltaTeta=Pi, from encoders to motors)
  deltaXEnc = -4*PI*RWheel/3/tourPulses*(cosTeta*deltaCounter3 - (0.5*cosTeta-sinTeta*0.866)*deltaCounter1 - (sinTeta*0.866+0.5*cosTeta)*deltaCounter2);
  deltaYEnc = 4*PI*RWheel/3/tourPulses*(-sinTeta*deltaCounter3 + (0.866*cosTeta+0.5*sinTeta)*deltaCounter1 - (0.866*cosTeta-0.5*sinTeta)*deltaCounter2);
  deltaTetaEnc = 2*PI/3/tourPulses*(deltaCounter1 + deltaCounter2 + deltaCounter3)*RWheel/RFreeRobot;
  
  XEnc = X + deltaXEnc;  // in mm
  YEnc = Y + deltaYEnc;
  tetaEnc = teta + deltaTetaEnc;  // in radians
//  Serial.println(X);
//  Serial.println(Y);
//  Serial.println(teta);
//  Serial.println(counterEncoder1);
//  Serial.println(counterEncoder2);
//  Serial.println(counterEncoder3);

  
  
  VitXEnc = deltaXEnc*encoderSampleFreq;
  VitYEnc = deltaXEnc*encoderSampleFreq;
  VitTetaEnc = deltaTetaEnc*encoderSampleFreq; 
  
  #ifdef ENCODERS
  measurements[0] = XEnc;
  measurements[1] = YEnc;
  measurements[2] = tetaEnc;
  #endif
  
  #ifndef COMPUTE_PID  
    X = XEnc; // If no PID
    Y = YEnc;
    teta = tetaEnc; 
    VitX = VitXEnc;
    VitY = VitXEnc;
    VitTeta = VitTetaEnc;
  #endif
}

//Function called with frequence 1/PIDSampleTime to read AccX, AccY, VitTetaZ, VitTetaX and VitTetaY from 
//Inertial Measurement Unit
void readImu()
{
//  timeDebugT3 = micros();
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//  Serial.println(micros()-timeDebugT3);  
  AccX = (ax-3110.95)*0.0001211;
  AccY = (ay+3251.6)*0.0001219;
  AccZ = (az+6583.7)*0.0001182;
  VitTetaXG = (gx+242.87)*0.0152;
  VitTetaYG = (gy-24.02)*0.0152; // receive a byte as character
  VitTetaZG = (gz-29.97)*0.0152; // receive a byte as character
  
  #ifdef SLOPING_FLOOR
  measurements[3] = VitTetaXG;
  measurements[4] = VitTetaYG;
  measurements[5] = VitTetaZG;
  measurements[6] = AccX;
  measurements[7] = AccY;  
  measurements[8] = AccZ;
  #endif
  #ifndef SLOPING_FLOOR
  #ifdef IMU
  measurements[3] = AccX;
  measurements[4] = AccY;  
  measurements[5] = VitTetaZG;
  #endif
  #endif
}

void intT3(){
  #ifndef COMPUTE_PID
      coneRefreshSpeed();
      coneRefreshAcceleration();
      if(typeOfControl==CONTROLROBOTSPEED)
      { 
        VitXRobotR = LastVitXRobotR;
        VitYRobotR = LastVitYRobotR;
        VitTetaRobotR = LastVitTetaRobotR;
        vitRobotR2wR();
      }
      else if(typeOfControl==CONTROLCARTESIANSPEED)
      {
        VitXR = LastVitXR;
        VitYR = LastVitYR;
        VitTetaR = LastVitTetaR;
        vitR2wR();  
      }
  #endif
      
  #ifdef IMU
  
    readImu();
    
    #ifdef COMPUTE_KF
    computeFilter();
    #endif
  #endif
  
  #ifdef COMPUTE_PID
    computeAllPID();
  #endif
//  Serial.println(X);
//  Serial.println(Y);
//  Serial.println(teta);
  
 
  #ifndef COMPUTE_PID
    E1 = w1R*12/24;
    E2 = w2R*12/24;
    E3 = w3R*12/24;
  #endif
  
  #ifdef ENGINES
    setEngines();
  #endif
  
  #ifdef DEBUG
  //Serial.println(micros()-timeDebugT3);
  #endif
}

//Function called with period PIDSampleTime to calculate the output of the PIDS and set the engines to
//the new values calculated
ISR(TIMER3_COMPA_vect)          // timer 3 compare interrupt service routine
{ 
  #ifdef DEBUG
//  Serial.println("T3");
//  timeDebugT3 = micros();
  #endif
  flagIntT3 = 1;  
}

#ifdef COMMUNICATION_SPI

//SPI interrupt routine
ISR(SPI_STC_vect)
{
//  SPDR = 0x5A;
//  Serial.println("SPI");
//  SPDR = 0x5A;
//  
  if(flagSendData)
  {
    #ifdef DEBUG
    //Serial.println("OI");
    #endif
    if(spiPos < nbBytesSpi){
      SPDR = bufferSendData[spiPos++];
 
    }
    else{
      //Serial.println(bufferSendData[spiPos-1]);
      spiPos = 0;
      flagSendData = 0;

    }
  }
  else
  {
    byte c = SPDR;  // grab byte from SPI Data Register
    SPDR = 0;
    #ifdef DEBUG
    //Serial.println(c);
    #endif
    // add to spiPosfer if room
    spiBuffer [spiPos++] = c;
/*        
      // Last byte of all messages
      if (c == 128) 
      {
        process_it();
      }
*/
  }  
}  // end of interrupt routine SPI_STC_vect
    
 
// end of loop
// function that executes whenever data is received from master
// Protocol: 1st byte: type of command 
// 2nd byte (if required): Control type. After:
// if control in position points (type 0): nbpositions, Array of positions describing a trajectory 
// if control in cartesian speeds (type 1): robot speed in a cartesian reference
// if control in relative to the robot speeds (type 2): robot reference speed 
// if control in position with the next point (type 3): next point to go (x,y)
void process_it() 
{
  spiPos = 0;
  typeOfCommand = spiBuffer[spiPos++];
  switch(typeOfCommand)
  {
    case CHANGETYPEOFCONTROL:
    {
      typeOfControl = spiBuffer[spiPos++];
      switch(typeOfControl)
      {
       
        //read an array of positions that correspond to the trajectory the robot has to follow
        case CONTROLPOSITIONPOINTS:
        {
          #ifdef DEBUG
          Serial.println("Position Points");
          #endif
          int nbPoints = (uint8_t)spiBuffer[spiPos++];
          for(int i=0;i<nbPoints;i++)
          {
            for(int j=0;j<nbCoord;j++)
            {
              uint16_t result;
              readOnePositionSpi(result);
              arrayPositions[i][j] = result; 
            }
          }  
          indexPosition = 0;
          XR = arrayPositions[indexPosition][0];    
          YR = arrayPositions[indexPosition][1];
          TetaR = arrayPositions[indexPosition][2];
        }
        break;
        
        //read values of environement-absolute speeds and turn them into angular wheel speed
        case CONTROLCARTESIANSPEED:
        {
          #ifdef DEBUG
          //Serial.println("Cartesian Speed");
          #endif
          LastVitXR = ((int8_t)spiBuffer[spiPos++])*maxVitX/127;
          LastVitYR = ((int8_t)spiBuffer[spiPos++])*maxVitY/127;
          LastVitTetaR = ((int8_t)spiBuffer[spiPos++])*maxVitTeta/127;   
          #ifndef COMPUTE_PID
            intT3();
          #endif
        }
        break;
       
        //read values of relative-to-the-robot-reference speeds and turn them into angular wheel speed
        case CONTROLROBOTSPEED:
        {
          #ifdef DEBUG
          //Serial.println("Robot Speed");
          #endif
          LastVitXRobotR = ((int8_t)spiBuffer[spiPos++])*maxVitX/127;
          LastVitYRobotR = ((int8_t)spiBuffer[spiPos++])*maxVitY/127;
          LastVitTetaRobotR = ((int8_t)spiBuffer[spiPos++])*maxVitTeta/70;
          #ifndef COMPUTE_PID
            intT3();
          #endif
        }
        break;
        
        //read values of the next position the robot has to go 
        case CONTROLPOSITIONNEXTPOINT:
        {
            #ifdef DEBUG
            Serial.println("Next Point");
            #endif
            int16_t result;
            readOnePositionSpi(result);
            XR = (double) result; 
            readOnePositionSpi(result);
            YR = (double) result; 
            TetaR = (double) spiBuffer[spiPos++];
            vitRobotR2wR();
        }
        break;       
      }  
    }
    break;
    
    // Read initial orientation of robot
    case RECEIVEORIENTATION: 
    {
      #ifdef DEBUG
      //Serial.println("Receiving orientation");
      #endif
      teta = ((int8_t)spiBuffer[spiPos++])*3.142/127;    //Initial orientation of robot 
      #ifdef DEBUG
      //Serial.println(teta);
      #endif
    }
    break;
    
    //Turn on engines with command (3)
    case ENABLEENGINES: 
    {
      enableEngines();
      #ifdef DEBUG
      //Serial.println("Enable Engines");
      //Serial.println("Enable Timer Interrupts");
      #endif
//      enableTimerInterrupts();
    } 
    break;
    
    //Turn off engines with command (4)
    case DISABLEENGINES: 
    {
      disableEngines();
      #ifdef DEBUG
//      Serial.println("Disable Engines");
//      Serial.println("Enable Timer Interrupts");
      #endif
//      disableTimerInterrupts();
    }
    break;
     
    //Send required information with command (5)
    case SENDDATA: 
    {
      #ifdef DEBUG
//      Serial.println("Send Data");
      //unsigned long timeTest = micros(); 
//      Serial.print("X ");
//      Serial.println(X);
//      Serial.print("Y ");
//      Serial.println(Y);
//      Serial.print("teta ");
//      Serial.println(teta);
//      Serial.print("encoder1 ");
//      Serial.println(counterEncoder1);
//      Serial.print("encoder2 ");
//      Serial.println(counterEncoder2);
//      Serial.print("encoder3 ");
//     Serial.println(counterEncoder3);
//     Serial.println(distanceObstacles[0]);
//     Serial.println(distanceObstacles[1]);
//     Serial.println(distanceObstacles[2]);
//     Serial.println(distanceObstacles[3]);
//     Serial.println(distanceObstacles[4]);
//     Serial.println(distanceObstacles[5]);
      #endif
      
      bufferSendData[0] = 10;
      
      byte* ptr = (byte*) &X;   
      bufferSendData[1] = *ptr;
      bufferSendData[2] = *(ptr+1);
      bufferSendData[3] = *(ptr+2);
      bufferSendData[4] = *(ptr+3);
  
      ptr = (byte*) &Y;
      bufferSendData[5] = *ptr;
      bufferSendData[6] = *(ptr+1);
      bufferSendData[7] = *(ptr+2);
      bufferSendData[8] = *(ptr+3);
      
      ptr = (byte*) &teta;
      bufferSendData[9] = *ptr;
      bufferSendData[10] = *(ptr+1);
      bufferSendData[11] = *(ptr+2);
      bufferSendData[12] = *(ptr+3);
//      
      ptr = (byte*) &counterEncoder1;   
      bufferSendData[13] = *ptr;
      bufferSendData[14] = *(ptr+1);
      bufferSendData[15] = *(ptr+2);
      bufferSendData[16] = *(ptr+3);
  
      ptr = (byte*) &counterEncoder2;
      bufferSendData[17] = *ptr;
      bufferSendData[18] = *(ptr+1);
      bufferSendData[19] = *(ptr+2);
      bufferSendData[20] = *(ptr+3);
      
      ptr = (byte*) &counterEncoder3;
      bufferSendData[21] = *ptr;
      bufferSendData[22] = *(ptr+1);
      bufferSendData[23] = *(ptr+2);
      bufferSendData[24] = *(ptr+3);
      
//      ptr = (byte*) &VitX;
//      bufferSendData[13] = *ptr;
//      bufferSendData[14] = *(ptr+1);
//      bufferSendData[15] = *(ptr+2);
//      bufferSendData[16] = *(ptr+3);
//      
//      ptr = (byte*) &VitY;
//      bufferSendData[17] = *ptr;
//      bufferSendData[18] = *(ptr+1);
//      bufferSendData[19] = *(ptr+2);
//      bufferSendData[20] = *(ptr+3);   
//      
//      ptr = (byte*) &VitTeta;
//      bufferSendData[21] = *ptr;
//      bufferSendData[22] = *(ptr+1);
//      bufferSendData[23] = *(ptr+2);
//      bufferSendData[24] = *(ptr+3);    
      
      #ifdef ULTRASONS
      ptr = (byte*) distanceObstacles;
      bufferSendData[25] = *ptr;
      bufferSendData[26] = *(ptr+1);
      bufferSendData[27] = *(ptr+2);
      bufferSendData[28] = *(ptr+3);
      bufferSendData[29] = *(ptr+4);
      bufferSendData[30] = *(ptr+5);
      #endif
      
      bufferSendData[nbBytesSpi-1] = 10;
      
      flagSendData = 1;
      
      #ifdef DEBUG
      //Serial.println("time");
      //Serial.print(micros()-timeTest);
      #endif
      
    }
     break;

    
     //Reset reference position (6)
    case RESETREF: 
    {      
      #ifdef DEBUG
//      Serial.println("Reset Reference Position");
      #endif
      X=0;
      Y=0;
      teta=0;
      XEnc = 0;
      YEnc=0;
      tetaEnc=0;
    }
    break; 
    
    //Turn off sonic sensors (7)
    case OFFULTRASON: 
    {      
      #ifdef DEBUG
//      Serial.println("Turn off ultrasons");
      #endif
      TIMSK1 &= !(1 << OCIE1A); //disable timer interrupts
    }
    break;
   
    //Change to hard reaction to obstacles avoidance (8)
    case ULTRASONHARD: 
    {      
      #ifdef DEBUG
//      Serial.println("Hard ultrason");
      #endif
      sonicProtectionStatus = HARDREACTION;
    }
    break;
   
   //Change to soft reaction to obstacles avoidance (9)
    case ULTRASONSOFT: 
    {      
      #ifdef DEBUG
//      Serial.println("Soft ultrasons");
      #endif
      sonicProtectionStatus = SOFTREACTION;
    }
    break; 
  }
  spiPos = 0;  
}   
#endif
