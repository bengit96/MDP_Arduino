#include "EnableInterrupt.h"
#include "DualVNH5019MotorShield.h"
#include "PID.h"
#include "Sensor.h"
#include "Movement.h"
#include "math.h"

//Movement pin
#define enA1 3 // Left
#define enB1 5 // Left
#define enA2 11 // Right
#define enB2 13 // Right

// Sensors
#define ps1 A0
#define ps2 A1
#define ps3 A2
#define ps4 A3
#define ps5 A4
#define ps6 A5

// change when kp,ki,kd is found

 //battery 21
//old
/*
float lkP = 1.09088;
float lkI = 0;//lkP/ (2 * 0.0627);
float lkD = 0.0669 * lkP;// * 0.5; 

float rkP = 1.07304;
float rkI = 0;//rkP/ (2 * 0.0594);
float rkD = 0.0813 * rkP; //* 0.5;
*/

/*
float lkP = 1.3688;
float lkI = 0;//lkP/ (2 * 0.0627);
float lkD = 0.0517 * lkP;// * 0.5; 

float rkP = 1.2444;
float rkI = 0;//rkP/ (2 * 0.0594);
float rkD = 0.0507 * rkP; //* 0.5;
*/

  //battery 20

//6.22 - 6.24
//old
/*
float lkP = 1.34272;
float lkI = 0;//lkP/ (2 * 0.0502);
float lkD = 0.0541 * lkP;// * 0.5; 

float rkP = 1.19776;  
float rkI = 0;//rkP/ (2 * 0.0594);
float rkD = 0.0525 * rkP; //* 0.5;
*/

float lkP = 1.24564; 
float lkI = 0;//lkP/ (2 * 0.0502);
float lkD = 0.0546 * lkP;// * 0.5; 

float rkP = 1.20512;  
float rkI = 0;//rkP/ (2 * 0.0594);
float rkD = 0.0492 * rkP; //* 0.5;


DualVNH5019MotorShield md;
Movement mv(enA1,enB1,enA2,enB2);
PID pid(lkP,lkI,lkD,rkP,rkI,rkD);
Sensor sensor;

static volatile unsigned long ltime_tmp = 0; // volatile will tell the compiler that the value must be checked every time
static volatile unsigned long rtime_tmp = 0;  



double sensorCal(int sensor_pin){
  double raw = analogRead(sensor_pin);
  delay(1000);
  return raw;
}


void leftmotor(){
  mv.ltickIncrement();
}

void rightmotor(){
  mv.rtickIncrement();
}



long l_speed = 0;
long r_speed = 0;

int tmp = 0;
void setup(){
  Serial.begin(115200);  
  float rpm = 70; 
  l_speed = pid.convertLSpeed(rpm); // change this functions based on gradient found and y intercept
  r_speed = pid.convertRSpeed(rpm); // change this functions based on gradient found and y intercept
  md.init();
  mv.init();
  enableInterrupt(mv.pinA1, leftmotor, RISING);
  enableInterrupt(mv.pinB2, rightmotor, RISING);
  
}

//gf05r83f06l83f02r83f03l83f05r83f03l83f05

int angle = 0;
int gridNum = 0;
int last =0;
void loop() {
  
  while (Serial.available() > 0){
    String data = Serial.readStringUntil('\n');
    char actions = data.charAt(0); 
    switch (actions) { 
      case 'k':
        md.setBrakes(300,300);
        break;
      case 'j':
        md.setSpeeds(-300,300);
        break;
      case 'p':
        //mv.tickCal(300,md);
        //delay(1000);
        break;
      case 's':
          Serial.print(sensor.LBDistance(3)); Serial.print(" ");
          Serial.print(sensor.LFDistance(3)); Serial.print(" ");
          Serial.print(sensor.FLDistance(1)); Serial.print(" ");
          Serial.print(sensor.FMDistance(1)); Serial.print(" ");
          Serial.print(sensor.FRDistance(1)); Serial.print(" ");    
          Serial.println(sensor.RDistance(1));
          break;
      case 'c':
          mv.wallHugging(l_speed, r_speed, md ,pid ,sensor);
          break;
      case 'f': 
          gridNum = data.substring(1).toInt();
          mv.moveForward(l_speed, r_speed, md, pid, gridNum,sensor);
          break;
      case 'l':      
          angle = data.substring(1).toInt();
          mv.moveLeft(l_speed,r_speed,md,pid,angle,sensor,0);
          break;
      case 'r':
          angle = data.substring(1).toInt();
          mv.moveRight(l_speed,r_speed,md,pid,angle,sensor,0);
          break;
      case 'g':
        for(int i = 1; i < data.length();i=i+3){
          //delay(200);
          last = 0;
          char gactions = data.charAt(i);
          gridNum = data.substring(i+1,i+3).toInt();
          if(i +3 == data.length()){ // if last command
            last = 1;
          }
          switch(gactions){
            case 'c':
              mv.wallHugging(l_speed, r_speed, md ,pid ,sensor);
              break;
            case 'f':
              tmp = mv.moveForwardGoal(l_speed, r_speed, md, pid, gridNum,sensor,0,last); // 0 = horizonal condition
              if(tmp == -1){
                Serial.println(data.substring(1,i+2)); //output movements
                i = data.length(); // end loop                
              }
              break;
            case 'h':
              tmp = mv.moveForwardGoal(l_speed, r_speed, md, pid, gridNum,sensor,1,last); // 1= horizontal condition
              if(tmp == -1){
                Serial.println(data.substring(1,i+2)); //output movements
                i = data.length(); // end loop         
                Serial.print(sensor.LBDistance(3)); Serial.print(" ");
                Serial.print(sensor.LFDistance(3)); Serial.print(" ");
                Serial.print(sensor.FLDistance(1)); Serial.print(" ");
                Serial.print(sensor.FMDistance(1)); Serial.print(" ");
                Serial.print(sensor.FRDistance(1)); Serial.print(" ");    
                Serial.println(sensor.RDistance(1));  
              }
              break;
            case 'l':
              mv.moveLeft(l_speed,r_speed,md,pid,91,sensor,1);
              break;
            case 'r':
              mv.moveRight(l_speed,r_speed,md,pid,91.5,sensor,1);
              break;
		        }
        }
        break;
      default: 
        Serial.println("fail");
        break;
    }

  }
}
