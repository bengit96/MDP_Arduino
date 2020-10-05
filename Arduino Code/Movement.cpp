#include "DualVNH5019MotorShield.h"
#include "Movement.h"
#include <math.h>


float lm = -2.8684; //battery21 
float lc = -34.3793;// battery21

float rm =   2.9334; //battery21 
float rc = 33.2671; //battery21 


 //6.24 batt 20
/*
float lm = -2.8464; //battery 20
float lc = -35.1899; //battery 20

float rm = 3.1557; //battery 20
float rc = 33.0283; //battery 20
*/

Movement::Movement(float lP,float lI, float lD, float rP,float rI, float rD){

	
  lkp = lP;
  lki = lI;
  lkd = lD;
  rkp = rP;
  rki = rI;
  rkd = rD;
	/*
  lk1 = lP + lI + lD;
	lk2 = -lP - 2*lD;
	lk3 = lD;
  rk1 = rP + rI + rD;
  rk2 = -rP - 2*rD;
  rk3 = rD;
  */
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
  float u = currentRPM + lkp * currentError + lkd * (currentError - previousLError);
  
  if( u <= 0 ){
    u = 0;
  }
  

  if(u >= 120){
    u = 120;
  }
  float returnSpeed = convertLSpeed(u);
  previousLError = currentError;
  return returnSpeed;
}

float Movement::computeR(long setRSpeed, unsigned long rtime){
  float currentRPM = (pow(10,6) * 60 /rtime )/ 562.25;
  //float currentSpeed = convertRSpeed(rpm);//how to use ticks. they said to not use pulsein for feedbacks for pid
  float setRRPM = convertRRPM(setRSpeed);
  float currentError = (setRRPM-currentRPM);

  float u = currentRPM + rkp * currentError + rkd * (currentError - previousRError);
  //float u = currentRPM + rkp * currentError + (rkd * (currentError-previousRError)) + rki * RErrors; //random online method
  //Serial.print("rerrors");Serial.println(RErrors);
  if( u >= 120 ){
    u = 120;
  }
  
  if( u <= 0 ){
    u = 0;
  } 
   
  float returnSpeed = convertRSpeed(u);
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
