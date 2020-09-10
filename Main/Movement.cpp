#include "DualVNH5019MotorShield.h"
#include "Movement.h"
#include <math.h>

Movement::Movement(int pA1,int pB1, int pA2,  int pB2, float lP,float lI, float lD, float rP,float rI, float rD){
	//Left
	pinA1 = pA1;
	pinB1 = pB1;
	//Right
	pinA2 = pA2;
	pinB2 = pB2;
	lKp = lP;
	lKi = lI;
	lKd = lD;
  rKp = rP;
  rKi = rI;
  rKd = rD;
	previousLSpeed = 0;
	previousLError = 0;
	previousLError2 = 0;
	previousRSpeed = 0;
	previousRError = 0;
	previousRError2 = 0;
  distanceL = 0;
  distanceR = 0;
  distanceTraversed = 0;
}


float Movement::computeL(long setLSpeed, unsigned long ltime){
  float currentRPM = (pow(10,6) * 60 /ltime )/ 562.25;
  float setLRPM = convertLRPM(setLSpeed); 
  long currentError = setLRPM - currentRPM;
  float k1 = lKp + lKi + lKd;
  float k2 = -lKp - 2*lKd;
  float k3 = lKd;
  //no queue like feature
  if(errorL[0] == -1){ 
    errorL[0] = currentError;
  }else if (errorL[1] == -1){
    errorL[1] = currentError;
  }else{
    previousLError2 = errorL[0];
    errorL[0] = errorL[1];
    errorL[1] = currentError;
  }
  //float u = currentRPM + k1 * currentError + k2 * previousLError + k3 * previousLError2;
  
  float u = currentRPM + k1 * currentError + k2 * previousLError + k3 * previousLError2;
  if( u <= setLRPM ){
    u = setLRPM;
  }
  if(u >= 120){
    u = 120;
  }
  float returnSpeed = convertLSpeed(u);
  /*
  Serial.print("currentRPM");Serial.println(currentRPM);
  Serial.print("k1 * current error");Serial.println(k1 * currentError);
  Serial.print("k2 * previous error");Serial.println(k2 * previousLError);
  Serial.print("k2 * previous error");Serial.println(k2 * previousLError2);
  Serial.print("return speed l ");Serial.println(returnSpeed);
  /*
  Serial.print("lk1");Serial.println(k1);
  Serial.print("lk2");Serial.println(k2);
  Serial.print("lk3");Serial.println(k3);
  Serial.print("u");Serial.println(u);
  Serial.print("previouslerror");Serial.println(previousLError);
  Serial.print("previouslerror2");Serial.println(previousLError2);
  Serial.print("l current error");Serial.println(currentError);
  Serial.print("l current rpm");Serial.println(currentRPM);
  Serial.print("l output rpm");Serial.println(u);
  */
  previousLError = currentError;
  return returnSpeed;
}

float Movement::computeR(long setRSpeed, unsigned long rtime){
  float currentRPM = (pow(10,6) * 60 /rtime )/ 562.25;
  //float currentSpeed = convertRSpeed(rpm);//how to use ticks. they said to not use pulsein for feedbacks for pid
  float setRRPM = convertRRPM(setRSpeed);
  float k1 = rKp + rKi + rKd;
  float k2 = -rKp - 2*rKd;
  float k3 = rKd;
  float currentError = (setRRPM-currentRPM);

  //no queue like feature
  if(errorR[0] == -1){ 
    errorR[0] = currentError;
  }else if (errorR[1] == -1){
    errorR[1] = currentError;
  }else{
    previousRError2 = errorR[0];
    errorR[0] = errorR[1];
    errorR[1] = currentError;
  }

  float u = currentRPM + k1 * currentError + k2 * previousRError + k3 * previousRError2;
  //float u = currentRPM + k1 * currentError + k2 * previousRError;// + k3 * previousRError2;
  if( u >= 120 ){
    u = 120;
  }
  if(u <= setRRPM){
    u = setRRPM;
  }
  float returnSpeed = convertRSpeed(u);
  /*
  Serial.print("currentRPM");Serial.println(currentRPM);
  Serial.print("set rpm");Serial.println(setRRPM);
  Serial.print("k1 * current error");Serial.println(k1 * currentError);
  Serial.print("k2 * previous error");Serial.println(k2 * previousRError);
  Serial.print("k3 * previous error");Serial.println(k3 * previousRError2);
  Serial.print("return speed r ");Serial.println(returnSpeed);
  
  /*
  Serial.print("currentRPM");Serial.println(currentRPM);
  Serial.print("k1 * current error");Serial.println(k1 * currentError);
  Serial.print("k2 * previous error");Serial.println(k2 * previousRError);
  Serial.print("previousrerror");Serial.println(previousRError);
  Serial.print("previousrerror2");Serial.println(previousRError2);  
  Serial.print("r current error");Serial.println(currentError);
  Serial.print("r current rpm");Serial.println(currentRPM);
  Serial.print("r output rpm");Serial.println(u);
  */
  previousRError = currentError;
  return returnSpeed;
}



float Movement::convertLSpeed(float rpm){ // negative speed. I dun think we need backwards movement?
  float m = -2.8568; //change according to gradient
  float c = -41.3512; // change according to y intercept
  return (rpm*m + c);
}

float Movement::convertRSpeed(float rpm){ //positive speed
  float m = 3.0944; //change according to gradient
  float c = 39.1839; // change according to y intercept
  return (rpm*m + c);
}

float Movement::convertLRPM(float lspeed){ //positive speed
  float m = -2.8568; //change according to gradient
  float c = -41.3512; // change according to y intercept
  float lrpm = (lspeed+c)/m;
  return lrpm;
}

float Movement::convertRRPM(float rspeed){ // negative speed. I dun think we need backwards movement?
  float m = 3.0944; //change according to gradient
  float c = 39.1839; // change according to y intercept
  float rrpm = (rspeed+c)/m;
  return rrpm;
}


void Movement::resetDistance(){
  distanceL = 0;
  distanceR = 0;
  distanceTraversed = 0;
}
