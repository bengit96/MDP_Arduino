#include "Sensor.h"
#include "DualVNH5019MotorShield.h"
#include "SharpIR.h"

SharpIR right_sensor(A0, 20150);
SharpIR left_back_sensor(A1, 1080);
SharpIR left_front_sensor(A2, 1080);
SharpIR front_middle_sensor(A3, 1080);
SharpIR front_left_sensor(A4, 1080);
SharpIR front_right_sensor(A5, 1080);


Sensor::Sensor(){
    
}

float Sensor::FMDistance(int method) {
  float distance;
  float x = front_middle_sensor.median_Voltage_Sampling();
  switch(method){
    case 1:
      distance = front_middle_sensor.distance() - 5.26;
      break;
    case 2:
      distance = (342249.6-32728.63*x)/(1+12763.97*x+328.3559*(pow(x,2))); 
      break;
    case 3:
      distance = (26.9723/x)-3.142919;
      break;
  }
  if(distance > 40 || distance < 0){
    return 40;
  } else {
    return distance;
  }
}

float Sensor::FLDistance(int method) {
  float distance;
  float x = front_left_sensor.median_Voltage_Sampling();
  switch(method){
    case 1:
      distance = front_left_sensor.distance() - 3.12;
      break;
    case 2:
      distance = (163.3724-5.125353*x)/(1+2.455257*x+2.906178*(pow(x,2)));
      break;
    case 3:
      //return distance = (30.23257/x)-5.944718;   
      distance = (31.05792/x)-6.114549; 
      break;
  }
  if(distance > 40 || distance < 0){
    return 40;
  } else{
    return distance;
  }
}

float Sensor::FRDistance(int method) {
  float distance;
  float x = front_right_sensor.median_Voltage_Sampling();
  switch(method){
    case 1:
      distance = front_right_sensor.distance() - 3.12;
      break;
    case 2:
      distance = (228550.6-46173.84*x)/(1+7737.163*x-259.7944*(pow(x,2)));
      break;
    case 3:
      distance = (31.05792/x)-6.114549;    
      break;
  }
  if(distance > 40 || distance < 0){
    return 40;
  } else{
    return distance;
  }
}

float Sensor::LFDistance(int method) {
  float distance;
  float x = left_front_sensor.median_Voltage_Sampling();
  switch(method){
    case 1:
      return distance = left_front_sensor.distance() - 5.34;
      break;
    case 2:
      return distance = (64608.62-10629.52*x)/(1+1970.039*x+384.1247*(pow(x,2)));
    case 3:
      distance = (32.2953/x)-7.67349;   
      if(distance > 40 || distance < 0){
        return 40;
      } else{
        return distance;
      }
  }
  return -1;
}

float Sensor::LBDistance(int method) {
  float distance;
  float x = left_back_sensor.median_Voltage_Sampling();
  switch(method){
    case 1:
      return distance = left_back_sensor.distance() - 5.64;
    case 2:
      return distance = (64608.62-10629.52*x)/(1+1970.039*x+384.1247*(pow(x,2)));
    case 3:
      distance = (30.68132/x)-7.074208;   
      if(distance > 40 || distance < 0){
        return 40;
      } else{
        return distance;
      }
  }
  return -1;
}

void floatsort(float a[], int size){
    float temp;
    for (int i=0; i<size; i++){
      for (int j=i; j>0; j--){
        if (a[j] < a[j-1]){
          temp = a[j];
          a[j] = a[j-1];
          a[j-1] = temp;
        }
        else
          break;
      }
    }
}

float Sensor::RDistance(int method) {
  float distance;
  
  float rs[10];
  
  for(int i = 0 ; i < 10; i++){
    delay(0.5);
    rs[i] = right_sensor.median_Voltage_Sampling();
    //Serial.println(rs[i]);
  }
  floatsort(rs,10);
  
  float x = rs[5];
  //float x = right_sensor.median_Voltage_Sampling();
  //Serial.println(rs[1]);
  //Serial.println(x);
  
  //float x  = right_sensor.median_Voltage_Sampling();
  switch(method){
    case 1:
      return distance = right_sensor.distance() - 14;
    case 2:
      return distance = (2.879996+0.618943*x)/(1+0.2174681*0.008219543*(pow(x,2)));
      //(2.879996+0.618943*x)/(1+0.2174681*x+0.008219543*(x^2))
    case 3:
      return distance = (84.14019/x)-28.26101;
      //-84.14019/(-28.26101-x)
  }
  return -1;
}
//convert to grid
int Sensor::convertLong(float distance) {
  if(0 <= distance && distance <= 11){
    return 0;
  } else if ( 11 < distance && distance <= 18){ 
    return 1;
  } else if ( 18 < distance && distance <= 28){ 
    return 2;
  } else if ( 28 < distance && distance <= 38){ 
    return 3;
  } else if ( 38 < distance && distance <= 48){ 
    return 4;
  } else{
    return 5;
  } 
}

//convert to grid
int Sensor::convertShort(float distance) {
  if( 0 <= distance && distance <= 10){
    return 0;
  } else if ( 10 < distance && distance <= 19){
    return 1;
  } else if ( 19 < distance && distance <= 29){
    return 2;
  } else{
    return 3;
  }
}

void Sensor::caliSensor() {
  Serial.println("calibration");
  Serial.print("Left Back Sensor: "); Serial.println(LBDistance(3));
  Serial.print("Left Front Sensor: "); Serial.println(LFDistance(3));
  Serial.print("Front Left Sensor: "); Serial.println(FLDistance(1));
  Serial.print("Front Middle Sensor: "); Serial.println(FMDistance(1));
  Serial.print("Front Right Sensor: "); Serial.println(FRDistance(1));
  Serial.print("Right Sensor: "); Serial.println(RDistance(1));
}
