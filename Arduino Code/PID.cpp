#include "DualVNH5019MotorShield.h"
#include "PID.h"
#include <math.h>

/*
float lm = -2.5818; //battery21 
float lc = -30.1993; // battery21

float rm =  2.7421; //battery21 
float rc = 25.6292; //battery21 
*/
 //6.24 batt 20

float lm = -2.7249; //battery 20
float lc = -15.4212; //battery 20

float rm = 2.7530; //battery 20
float rc = 15.3233; //battery 20


PID::PID(float lP,float lI, float lD, float rP,float rI, float rD){

	
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


float PID::computeL(long setLSpeed, unsigned long ltime, DualVNH5019MotorShield md, int fp){
  float currentRPM = (pow(10,6) * 60 /ltime )/ 562.25;
  float setLRPM = convertLRPM(setLSpeed); 
  long currentError = setLRPM - currentRPM;
  float u  = 0;
  if(!fp){
    u = currentRPM + lkp * currentError + lkd * (currentError - previousLError);
  }else{
    //batt 20
    u = currentRPM + 1.24064 * currentError + 0.0546*1.24064 * (currentError - previousLError);
    //batt 21
    //u = currentRPM + 1.4058 * currentError +  0.0517 *1.4058 * (currentError - previousLError);
  }

  
  if( u <= 0 ){
    u = 0;
  }
  

  if(u >= 140){
    u = 140;
  }
  float returnSpeed = convertLSpeed(u);
  previousLError = currentError;
  return returnSpeed;
}

float PID::computeR(long setRSpeed, unsigned long rtime, DualVNH5019MotorShield md, int fp){
  float currentRPM = (pow(10,6) * 60 /rtime )/ 562.25;
  //float currentSpeed = convertRSpeed(rpm);//how to use ticks. they said to not use pulsein for feedbacks for pid
  float setRRPM = convertRRPM(setRSpeed);
  float currentError = (setRRPM-currentRPM);
  float u =0;
  if(!fp){
    u = currentRPM + rkp * currentError + rkd * (currentError - previousRError);
  }else{
    //batt 20
    u = currentRPM + 1.20512 * currentError + 0.0492 * 1.20512* (currentError - previousLError);
    //batt 21
    //u = currentRPM + 1.2444 * currentError + 0.0507 * 1.2444 * rkd * 1.2444 * (currentError - previousLError);
  }
  //float u = currentRPM + rkp * currentError + (rkd * (currentError-previousRError)) + rki * RErrors; //random online method
  //Serial.print("rerrors");Serial.println(RErrors);
  if( u >= 138 ){
    u = 138;
  }
  
  if( u <= 0 ){
    u = 0;
  } 
   
  float returnSpeed = convertRSpeed(u);
  previousRError = currentError;
  return returnSpeed;
}



float PID::convertLSpeed(float rpm){ // negative speed
  return (rpm*lm + lc);
}

float PID::convertRSpeed(float rpm){ //positive speed
  return (rpm*rm + rc);
}

float PID::convertLRPM(float lspeed){ //negative speed
  float lrpm = (lspeed-lc)/lm;
  return lrpm;
}

float PID::convertRRPM(float rspeed){ // postivie speed
  float rrpm = (rspeed-rc)/rm;
  return rrpm;
}


void PID::resetDistance(){
  distanceL = 0;
  distanceR = 0;
  distanceTraversed = 0;
}
