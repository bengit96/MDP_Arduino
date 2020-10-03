#include "DualVNH5019MotorShield.h"
#include "Movement.h"
#include <math.h>

/*
float lm = -2.8684; //battery21 
float lc = -34.3793;// battery21

float rm =   2.9334; //battery21 
float rc = 33.2671; //battery21 
*/

 //6.24 batt 20

float lm = -2.8464; //battery 20
float lc = -35.1899; //battery 20

float rm = 3.1557; //battery 20
float rc = 33.0283; //battery 20


Movement::Movement(float lP,float lI, float lD, float rP,float rI, float rD){

	
  lkp = lP;
  lki = lI;
  lkd = lD;
  rkp = rP;
  rki = rI;
  rkd = rD;
	lk1 = lP + lI + lD;
	lk2 = -lP - 2*lD;
	lk3 = lD;
  rk1 = rP + rI + rD;
  rk2 = -rP - 2*rD;
  rk3 = rD;
	previousLSpeed = 0;
	previousLError = 0;
	previousLError2 = 0;
	previousRSpeed = 0;
	previousRError = 0;
	previousRError2 = 0;
  distanceL = 0;
  distanceR = 0;
  distanceTraversed = 0;
  LErrors = 0;
  RErrors = 0;
}


/*
float lm = -2.6935; //battery21 
float lc = -35.4937;// battery21

float rm =   2.8675; //battery21 
float rc = 22.7487; //battery21 
*/


float Movement::computeL(long setLSpeed, unsigned long ltime){
  float currentRPM = (pow(10,6) * 60 /ltime )/ 562.25;
  float setLRPM = convertLRPM(setLSpeed); 
  long currentError = setLRPM - currentRPM;
  LErrors += currentError;
  /*
  if(errorL[0] == -1){ 
    errorL[0] = currentError;
  }else if (errorL[1] == -1){
    errorL[1] = currentError;
  }else{
    previousLError2 = errorL[0];
    errorL[0] = errorL[1];
    errorL[1] = currentError;
  }
  */
  //float u = currentRPM + lk1 * currentError + lk2 * previousLError + lk3 * previousLError2;
  
  //float u = currentRPM + lkp * currentError + lkd * (currentError - previousLError) + lki * LErrors;
  float u = currentRPM + lkp * currentError + lkd * (currentError - previousLError);
   //float u = currentRPM + lkp * currentError + (lkd * (currentError-previousLError)) + lki * LErrors; //random online method
  //Serial.print("lerrors");Serial.println(LErrors);
 /*
  if( u <= setLRPM ){
    u = setLRPM;
  }
  */
  
  if( u <= 0 ){
    u = 0;
  }
  
 
  /*
  Serial.print("lk1");Serial.println(k1);
  Serial.print("lk2");Serial.println(k2);
  Serial.print("lk3");Serial.println(k3);
  Serial.print("u");Serial.println(u);
  Serial.print("previouslerror");Serial.println(previousLError);
  Serial.print("previouslerror2");Serial.println(previousLError2);
  Serial.print("l current error");Serial.println(currentError);
  Serial.print("l current rpm");Serial.println(currentRPM);
  */
  //Serial.print("l output rpm");Serial.println(u);

  if(u >= 120){
    u = 120;
  }
  float returnSpeed = convertLSpeed(u);
  //Serial.print("currentRPM");Serial.println(currentRPM);
  //Serial.print("k1 * current error");Serial.println(lk1 * currentError);
  //Serial.print("k2 * previous error");Serial.println(lk2 * previousLError);
  //Serial.print("k3 * previous error");Serial.println(lk3 * previousLError2);
  //Serial.print("return speed l ");Serial.println(returnSpeed);
  previousLError = currentError;
  return returnSpeed;
}

float Movement::computeR(long setRSpeed, unsigned long rtime){
  float currentRPM = (pow(10,6) * 60 /rtime )/ 562.25;
  //float currentSpeed = convertRSpeed(rpm);//how to use ticks. they said to not use pulsein for feedbacks for pid
  float setRRPM = convertRRPM(setRSpeed);
  float currentError = (setRRPM-currentRPM);
  RErrors+= currentError;
  /*
  if(errorR[0] == -1){ 
    errorR[0] = currentError;
  }else if (errorR[1] == -1){
    errorR[1] = currentError;
  }else{
    previousRError2 = errorR[0];
    errorR[0] = errorR[1];
    errorR[1] = currentError;
  }
  */
  //float u = currentRPM + rk1 * currentError + rk2 * previousRError + rk3 * previousRError2;
  //float u = currentRPM + rkp * currentError + rkd * (currentError - previousRError) + rki * RErrors;
  float u = currentRPM + rkp * currentError + rkd * (currentError - previousRError);
  //float u = currentRPM + rkp * currentError + (rkd * (currentError-previousRError)) + rki * RErrors; //random online method
  //Serial.print("rerrors");Serial.println(RErrors);
  if( u >= 120 ){
    u = 120;
  }
  
  if( u <= 0 ){
    u = 0;
  } 
   
  /*
  if(u <= setRRPM){
    u = setRRPM;
  }
  */
  float returnSpeed = convertRSpeed(u);
  //Serial.print("currentRPM");Serial.println(currentRPM);
    //Serial.print("return speed r ");Serial.println(returnSpeed);
  //Serial.print("r output rpm");Serial.println(u);

  /*
  Serial.print("set rpm");Serial.println(setRRPM);
  Serial.print("k1 * current error");Serial.println(rk1 * currentError);
  Serial.print("k2 * previous error");Serial.println(rk2 * previousRError);
  Serial.print("k3 * previous error");Serial.println(rk3 * previousRError2);
  Serial.print("return speed r ");Serial.println(returnSpeed);
  */
  previousRError = currentError;
  return returnSpeed;
}



float Movement::convertLSpeed(float rpm){ // negative speed
  return (rpm*lm + lc);
}

float Movement::convertRSpeed(float rpm){ //positive speed
  return (rpm*rm + rc);
}

float Movement::convertLRPM(float lspeed){ //negative speed
  float lrpm = (lspeed-lc)/lm;
  return lrpm;
}

float Movement::convertRRPM(float rspeed){ // postivie speed
  float rrpm = (rspeed-rc)/rm;
  return rrpm;
}


void Movement::resetDistance(){
  distanceL = 0;
  distanceR = 0;
  distanceTraversed = 0;
}
