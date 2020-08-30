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
/*
float Movement::computeL(long setLSpeed, unsigned long ltime){
	float rpm = (pow(10,6) * 60 /ltime )/ 562.25;
	long currentSpeed = convertLSpeed(rpm);; //how to use ticks. they said to not use pulsein for feedbacks for pid
	long currentError = (abs(currentSpeed)-abs(setLSpeed));
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
	float u = currentSpeed + k1 * currentError + k2 * previousLError + k3 * previousLError2;
  if( u >= 0 ){
    u = 0;
  }
  if(u <= -400){
    u = -400;
  }
  Serial.println(k1);
  Serial.println(k2);
  Serial.println(k3);
	Serial.println(currentError);
  Serial.println(currentSpeed);
  Serial.println(u);
	previousLError = currentSpeed-setLSpeed;
	return u;
}
*/

float Movement::computeL(long setLSpeed, unsigned long ltime){
  float currentRPM = (pow(10,6) * 60 /ltime )/ 562.25;
  float m = -2.7814; //change according to gradient
  float c = -21.8053; // change according to y intercept
  float setLRPM = (setLSpeed+c)/m;  
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
  float u = currentRPM + k1 * currentError; //+ k2 * previousLError + k3 * previousLError2;
  if( u <= 0 ){
    u = 0;
  }
  if(u >= 130){
    u = 130;
  }
  float returnSpeed = convertLSpeed(u);
  
  Serial.println(k1);
  Serial.println(k2);
  Serial.println(k3);
  Serial.println(currentError);
  Serial.println(currentRPM);
  Serial.println(u);
  
  previousLError = currentError;
  return returnSpeed;
}

float Movement::computeR(long setRSpeed, unsigned long rtime){
  float currentRPM = (pow(10,6) * 60 /rtime )/ 562.25;
  //float currentSpeed = convertRSpeed(rpm);//how to use ticks. they said to not use pulsein for feedbacks for pid
  float m = 2.8291; //change according to gradient
  float c = 17.4258; // change according to y intercept
  float setRRPM = (setRSpeed+c)/m;
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

  float u = currentRPM + k1 * currentError;// + k2 * previousRError + k3 * previousRError2;
  if( u >= 130 ){
    u = 130;
  }
  if(u <= 0){
    u = 0;
  } 
  Serial.println(k1);
  float returnSpeed = convertRSpeed(u);
  previousRError = currentError;
  return returnSpeed;
}

/*
float Movement::computeR(long setRSpeed, unsigned long rtime){
  float rpm = (pow(10,6) * 60 /rtime )/ 562.25;
	float currentSpeed = convertRSpeed(rpm);//how to use ticks. they said to not use pulsein for feedbacks for pid
	float k1 = rKp + rKi + rKd;
	float k2 = -rKp - 2*rKd;
	float k3 = rKd;
	float currentError = (currentSpeed-setRSpeed);

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

	float u = currentSpeed + k1 * currentError + k2 * previousRError + k3 * previousRError2;
  if( u >= 400 ){
    u = 400;
  }
  if(u <= 0){
    u = 0;
  }	
	previousRSpeed = u;
	previousRError = currentError;
	return u;
}
*/

float Movement::convertLSpeed(float rpm){ // negative speed. I dun think we need backwards movement?
  float m = -2.7814; //change according to gradient
  float c = -21.8053; // change according to y intercept
  return (rpm*m + c);
}

float Movement::convertRSpeed(float rpm){ //positive speed
  float m = 2.8291; //change according to gradient
  float c = 17.4258; // change according to y intercept
  return (rpm*m + c);
}

void Movement::resetDistance(){
  distanceL = 0;
  distanceR = 0;
  distanceTraversed = 0;
}
/*
void Movement::moveForward(long setLSpeed, long setRSpeed, Encoder en, DualVNH5019MotorShield md){
  //resetDistance();
  md.setSpeeds(setLSpeed,setRSpeed);
	while(distanceTraversed <= 10){
	  Serial.println(ltime);
	  //long tmpl_speed = computeL(setLSpeed);
    //Serial.println(tmpl_speed);
    //delay(1000);
	}
	
	while(distanceTraversed <= 10){ // while havent reach distance, calibrate speed every 0.01seconds
		delay(0.05); // should delay so the speed don't keep changing. need to tweak to get best interval
		long tmpl_speed = computeL(setLSpeed);
		long tmpr_speed = computeR(setRSpeed);
    Serial.print("lspeed");Serial.println(tmpl_speed);
		Serial.print("rspeed");Serial.println(tmpr_speed);
		md.setSpeeds(tmpl_speed,tmpr_speed);
		distanceL += 6 * (en.getLticks()/562.25); //assuming wheelradius = 6
		distanceR += 6 * (en.getRticks()/562.25);
		Serial.println(distanceL);
    Serial.println(distanceR);
		en.resetTicks();
		distanceTraversed = (distanceL + distanceR)/2;
	  Serial.println(distanceTraversed);
	}
 
//resetDistance();
}
*/
