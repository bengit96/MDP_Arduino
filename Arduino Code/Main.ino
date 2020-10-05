#include "EnableInterrupt.h"
#include "DualVNH5019MotorShield.h"
#include "Movement.h"
#include "Sensor.h"
//#include "FastPID.h"
#include "Encoder.h"
#include "math.h"

//Encoder pin
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


float lkP = 1.9508;
float lkI = 0;//lkP/ (2 * 0.0502);
float lkD = 0.0582 * lkP;// * 0.5; 

float rkP = 1.73676;
float rkI = 0;//rkP/ (2 * 0.0594);
float rkD = 0.0544 * rkP; //* 0.5;

  //battery 20

//6.22 - 6.24
/*
float lkP = 1.5096;
float lkI = 0;
float lkD = (0.0422*lkP); 

float rkP = 1.25584;
float rkI = 0;
float rkD = (0.0428*rkP);
*/

DualVNH5019MotorShield md;
Encoder en(enA1,enB1,enA2,enB2);
Movement mv(lkP,lkI,lkD,rkP,rkI,rkD);
Sensor sensor;

static volatile unsigned long ltime_tmp = 0; // volatile will tell the compiler that the value must be checked every time
static volatile unsigned long rtime_tmp = 0;  

//gr33h12 //l33f05

double sensorCal(int sensor_pin){
  double raw = analogRead(sensor_pin);
  delay(1000);
  return raw;
}


void leftmotor(){
  en.ltickIncrement();
}

void rightmotor(){
  en.rtickIncrement();
}



long l_speed = 0;
long r_speed = 0;

int tmp = 0;
void setup(){
  Serial.begin(115200);  
  float rpm = 50; 
  l_speed = mv.convertLSpeed(rpm); // change this functions based on gradient found and y intercept
  r_speed = mv.convertRSpeed(rpm); // change this functions based on gradient found and y intercept
  md.init();
  en.init();
  enableInterrupt(en.pinA1, leftmotor, RISING);
  enableInterrupt(en.pinB2, rightmotor, RISING);
  
}

//gr33h05r33f03l33h01gl33f02r33h03l33f06

int angle = 0;
int gridNum = 0;

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
        //en.tickCal(300,md);
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
          en.wallHugging(l_speed, r_speed, md ,mv ,sensor);
          break;
      case 'f': 
          gridNum = data.substring(1).toInt();
          en.moveForward(l_speed, r_speed, md, mv, gridNum,sensor);
          break;
      case 'l':      
          angle = data.substring(1).toInt();
          en.moveLeft(l_speed,r_speed,md,mv,angle,sensor,0);
          break;
      case 'r':
          angle = data.substring(1).toInt();
          en.moveRight(l_speed,r_speed,md,mv,angle,sensor,0);
          break;
      case 'g':
        for(int i = 1; i < data.length();i=i+3){
          delay(500);
          char gactions = data.charAt(i);
          gridNum = data.substring(i+1,i+3).toInt();
          switch(gactions){
            case 'c':
              en.wallHugging(l_speed, r_speed, md ,mv ,sensor);
              break;
            case 'f':
              tmp = en.moveForwardGoal(l_speed, r_speed, md, mv, gridNum,sensor,0);
              if(tmp == -1){
                Serial.println(data.substring(1,i+2)); //output movements
                i = data.length(); // end loop                
              }
              break;
            case 'h':
              tmp = en.moveForwardGoal(l_speed, r_speed, md, mv, gridNum,sensor,1);
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
              en.moveLeft(l_speed,r_speed,md,mv,gridNum,sensor,1);
              break;
            case 'r':
              en.moveRight(l_speed,r_speed,md,mv,gridNum,sensor,1);
              break;
          }
        }
        break;
      default: 
        Serial.println("fail");
        break;
    }
     //Serial.println("end");
  // Every 10 grids will output back a sensor value
  }
}


  //en.wallHugging(l_speed, r_speed, md ,mv ,sensor);
  /*
  Serial.print("Left front distance");Serial.println(sensor.LFDistance(1));
  Serial.print("Left back distance");Serial.println(sensor.LBDistance(1));
  Serial.print("Front left distance");Serial.println(sensor.FLDistance(1));
  Serial.print("Front middle distance");Serial.println(sensor.FMDistance(1));
  Serial.print("Front right distance");Serial.println(sensor.FRDistance(1));
  Serial.print("Right distance");
  Serial.println(sensor.RDistance(1));
  */
  //delay(2000);
  //Serial.print("Right distance");Serial.println(sensor.RDistance(2));
  //Serial.print("Right distance");Serial.println(sensor.RDistance(3));
  //en.moveRight(l_speed,r_speed,md,mv,810); //720
  //en.moveRight(l_speed,r_speed,md,mv,405); //360
  //en.moveRight(l_speed,r_speed,md,mv,202); //180
  //en.moveRight(l_speed,r_speed,md,mv,90); //90
  //en.moveLeft(l_speed,r_speed,md,mv,810); //720
  //en.moveLeft(l_speed,r_speed,md,mv,405); //360
  //en.moveLeft(l_speed,r_speed,md,mv,200); //180
