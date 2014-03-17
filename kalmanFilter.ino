// Compute R - Covariance Matrix of measurements noise - Ã  faire 
void computeFilter()
{ 
  #ifdef SLOPING_FLOOR
  REAL state_transition[NSTATES*NSTATES]  = {1,  0,  0,  0,  0,  0,  KFSampleTime,  0,               0,                 0,               0,                 0,                 KFSampleTime*KFSampleTime/2,     0,                            0,
                                             0,  1,  0,  0,  0,  0,  0,             KFSampleTime,    0,                 0,               0,                 0,                 0,                               KFSampleTime*KFSampleTime/2,  0, 
                                             0,  0,  1,  0,  0,  0,  0,             0,               KFSampleTime,      0,               0,                 0,                 0,                               0,                            KFSampleTime*KFSampleTime/2
                                             0,  0,  0,  1,  0,  0,  0,             0,               0,                 KFSampleTime,    0,                 0,                 0,                               0,                            0,
			                     0,  0,  0,  0,  1,  0,  0,             0,               0,                 0,               KFSampleTime,      0,                 0,                               0,                            0,
			                     0,  0,  0,  0,  0,  1,  0,             0,               0,                 0,               0,                 KFSampleTime,      0,                               0,                            0,
			                     0,  0,  0,  0,  0,  0,  1,             0,               0,                 0,               0,                 0,                 KFSampleTime,                    0,                            0,
	                                     0,  0,  0,  0,  0,  0,  0,             1,               0,                 0,               0,                 0,                 0,                               KFSampleTime,                 0,
                                             0,  0,  0,  0,  0,  0,  0,             0,               1,                 0,               0,                 0,                 0,                               0,                            KFSampleTime,
                                             0,  0,  0,  0,  0,  0,  0,             0,               0,                 1,               0,                 0,                 0,                               0,                            0,
                                             0,  0,  0,  0,  0,  0,  0,             0,               0,                 0,               1,                 0,                 0,                               0,                            0,
                                             0,  0,  0,  0,  0,  0,  0,             0,               0,                 0,               0,                 1,                 0,                               0,                            0,
                                             0,  0,  0,  0,  0,  0,  0,             0,               0,                 0,               0,                 0,                 1,                               0,                            0,  
                                             0,  0,  0,  0,  0,  0,  0,             0,               0,                 0,               0,                 0,                 0,                               1,                            0,
                                             0,  0,  0,  0,  0,  0,  0,             0,               0,                 0,               0,                 0,                 0,                               0,                            1};
  #endif
  #ifndef SLOPING_FLOOR
  #ifdef IMU    
  REAL state_transition[NSTATES*NSTATES]  = {1., 0, 0, KFSampleTime, 0, 0, KFSampleTime*KFSampleTime/2, 0,
                             0,  1., 0, 0, KFSampleTime, 0, 0, KFSampleTime*KFSampleTime/2, 
                             0,  0,  1, 0, 0, KFSampleTime, 0, 0,
                             0,  0,  0, 1, 0, 0, KFSampleTime, 0,
			     0,  0,  0, 0, 1, 0, 0, KFSampleTime,
			     0,  0,  0, 0, 0, 1, 0, 0,
			     0,  0,  0, 0, 0, 0, 1, 0,
	                     0,  0,  0, 0, 0, 0, 0, 1 };
  #endif
  #ifndef IMU
  REAL state_transition[NSTATES*NSTATES]  = {1., 0, 0, KFSampleTime, 0, 0,
                             0,  1., 0, 0, KFSampleTime, 0, 
                             0,  0,  1, 0, 0, KFSampleTime,
                             0,  0,  0, 1, 0, 0,
			     0,  0,  0, 0, 1, 0,
			     0,  0,  0, 0, 0, 1};
  #endif 
  #endif
  
  // only position is measurable; not velocity
  matrix x = matrix(NSTATES,1,state); // initial state (location and velocity)
  matrix P = matrix(NSTATES,NSTATES, uncertainty); // 4x4 initial uncertainty
  matrix meas = matrix(NSEEN,1,measurements);
  
  matrix u = matrix( NSTATES, 1); // 4x1 external motion; set to 0.

	// u comes from what the vehicle is trying to do, e.g. accelerate.
  matrix F = matrix(NSTATES,NSTATES, state_transition); //  next state function
  
  #ifdef SLOPING_FLOOR
  REAL observable [NSTATES*NSEEN] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                                     0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                     0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                                     0, 0, 0, -sin(teta)*2*g*sin(tetaX)*cos(tetaX), -cos(teta)*2*g*sin(tetaY)*cos(tetaY), 0, 0, 0, 0, 0, 0, 0, cos(teta)*sin(tetaY), sin(teta)*sin(tetaX), 0,
                                     0, 0, 0, cos(teta)*2*g*sin(tetaX)*cos(tetaX),  -sin(teta)*2*g*sin(tetaY)*cos(tetaY), 0, 0, 0, 0, 0, 0, 0, sin(teta)*sin(tetaY), -cos(teta)*sin(tetaX),  0,
                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, cos(tetaX)+cos(tetaY)};
  #endif
  #ifndef SLOPING_FLOOR
  #ifdef IMU  
  #ifdef ENCODERS
  REAL observable [NSTATES*NSEEN] = {1, 0, 0, 0, 0, 0, 0, 0, 
                                     0, 1, 0, 0, 0, 0, 0, 0,
                                     0, 0, 1, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 1, 0, 0,
                                     0, 0, 0, 0, 0, 0, cos(teta), sin(teta),
                                     0, 0, 0, 0, 0, 0, sin(teta), -cos(teta)};
  #endif
  #ifndef ENCODERS
  REAL observable [NSTATES*NSEEN] = {0, 0, 0, 0, 0, 1, 0, 0,
                                     0, 0, 0, 0, 0, 0, cos(teta), sin(teta),
                                     0, 0, 0, 0, 0, 0, sin(teta), -cos(teta)};
  #endif
  #endif
  #ifndef IMU
  REAL observable [NSTATES*NSEEN] = {1, 0, 0, 0, 0, 0,
                                     0, 1, 0, 0, 0, 0,
                                     0, 0, 1, 0, 0, 0};
  #endif
  #endif
                    
  matrix H = matrix(NSEEN,NSTATES, observable); //  measurement function
  matrix R = matrix(NSEEN,NSEEN, measCovar);  //  measurement variance
  matrix I = matrix(NSTATES); // 4x4 identity matrix
  matrix Q = matrix(NSTATES,NSTATES, processCovar); 
 
 // prediction
  matrix Fx = F * x;
  matrix xNew = Fx + u;
  x = xNew;  // x = F*x + u
  matrix Ftrans = F.transpose();
  matrix PFt = P * Ftrans;
  matrix P2 = F * PFt;
  matrix P2_Q = P2 + Q;
  P =  P2_Q; // P = F * P * F.transpose() + Q;
  
  // measurement update
  // Arduino compiler gets confused on combining multiple matrix operations.
  matrix Hx =  H * x;
  matrix y =  meas - Hx; // y = Z- H * x
  matrix transp = H.transpose();
  matrix PHt = P * transp;
  matrix HPH = H * PHt;
  matrix S = HPH + R; // S = H * P * H.transpose() + R;
  matrix Sinv = S.inverse();
  matrix K = PHt * Sinv;// K = P * H.transpose()*S.inverse();
  matrix Ky = K * y;
  matrix x_Ky = x + Ky;
  x = x_Ky;
  matrix KH = K*H;
  matrix IKH = I-(KH);
  matrix Pnew = IKH * P; // (I-(K*H)) * P;  
  P = Pnew;
  
  // Put x into State and update variables
  x.values(state);
  X = state[0];
  Y = state[1];
  teta = state[2];
  VitX = state[3];
  VitY = state[4];
  VitTeta = state[5];
  #ifdef IMU
  AccX = state[6];
  AccY = state[7];
  #endif
  // Put P into Uncertainty.
  P.values(uncertainty);
}

