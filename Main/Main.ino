#include "EnableInterrupt.h"
#include "DualVNH5019MotorShield.h"
#include "Movement.h"
#include "Sensor.h"
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
/*
#define lkP 0.007578947368
#define lkI 0.09473684211
#define lkD 0.0001515789474

#define rkP 0.0007058823529
#define rkI 0.03529411765
#define rkD 0.000003529411765
*/

float lkP = 1.248; //fri(5.17984);//0.162808185;// 
float lkI = 0;//1.261647912;
float lkD = 0.0873 * lkP;//fri(0.0749*5.17984);//0.005144738647;

float rkP = 1.99696;//fri(4.14592); //5.4249438649;//
float rkI = 0;//5.788603819;
float rkD = 0.0456 * rkP;//fri(1.072*4.14592); //0.0100711696;


// Declaration
DualVNH5019MotorShield md;
Encoder en(enA1,enB1,enA2,enB2);
Movement mv(enA2,enB1,enA2,enB2,lkP,lkI,lkD,rkP,rkI,rkD);
Sensor sensor;

static volatile unsigned long ltime_tmp = 0; // volatile will tell the compiler that the value must be checked every time
static volatile unsigned long rtime_tmp = 0;  


void leftmotor(){
  en.ltickIncrement();
}

void rightmotor(){
  en.rtickIncrement();
}


//Temporarily to test
long l_speed = 0;
long r_speed = 0;


void setup(){
  Serial.begin(115200);  

  /*
  int RPM = 50;
  // Motor 1 baseline // left
  // rpm = (0.3712 * m1_speed) - 7.7114; //positive
  // rpm = (-0.3847 * m1_speed) - 7.7614; //negative
  int m1_speed = (RPM + 7.7614 )/ -0.3847 - 1.7;

  //Motor 2 baseline //right
  // rpm = (0.3422 * m2_speed) - 8.6711 // positive
  // rpm = (-0.344 * m2_speed) - 7.6516 // negative
  int m2_speed = (RPM + 8.6711)/ 0.3422;
  Serial.println(m1_speed);
  Serial.println(m2_speed);  
  */
  // Serial.println("Dual VNH5019 Motor Shield");
  md.init();
  en.init();
  enableInterrupt(en.pinA1, leftmotor, RISING);
  enableInterrupt(en.pinA2, rightmotor, RISING);
  int cal = 0; // set 1 if calibrating
  
  if(cal){    
    en.tickCal(300,md);
    delay(1000);
    exit(1);
  }else{
    Serial.println("not calibrating");
  }
  
  

  Serial.print("l_speed");Serial.println(l_speed); // put the speeds into moveForward function
  Serial.print("r_speed");Serial.println(r_speed);
  
}

float rpm;
void loop() {
  /*
  float rpm = 50; // change accordingly
  l_speed = mv.convertLSpeed(rpm); // change this functions based on gradient found and y intercept
  r_speed = mv.convertRSpeed(rpm); // change this functions based on gradient found and y intercept
  Serial.println(l_speed);
  Serial.println(r_speed);
  Serial.println("normal");
  Serial.println(); */
  //delay(2000);  
  //md.setSpeeds(-125,92);
  //delay(5000);
  
  //en.wallHugging(l_speed, r_speed, md ,mv ,sensor);
  /*
  Serial.print("Left front distance");Serial.println(sensor.LFDistance(1));
  Serial.print("Left back distance");Serial.println(sensor.LBDistance(1));
  Serial.print("Front left distance");Serial.println(sensor.FLDistance(1));
  Serial.print("Front middle distance");Serial.println(sensor.FMDistance(1));
  Serial.print("Front right distance");Serial.println(sensor.FRDistance(1));
  Serial.print("Right distance");Serial.println(sensor.RDistance(1)); */

  sensor.caliSensor();
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



  rpm = 50; // change accordingly
  l_speed = mv.convertLSpeed(rpm); 
  r_speed = mv.convertRSpeed(rpm); 
  //en.moveForward(l_speed,r_speed,md,mv,13,sensor);

  //en.wallHugging(l_speed, r_speed, md ,mv ,sensor);
  //en.moveForward(l_speed,r_speed,md,mv,8,sensor);
  //en.moveLoop(l_speed,r_speed,md,mv,sensor);
  //en.checkList1(l_speed,r_speed,md,mv,6,sensor); // move straight and object collison
  //en.checkList2(l_speed,r_speed,md,mv,6,sensor); //extension beyond basics
 
  delay(1000);   
  /*
  delay(1000);
  en.moveLeft(l_speed,r_speed,md,mv,90); //90
  en.wallHugging(l_speed, r_speed, md ,mv ,sensor);
  en.moveForward(l_speed,r_speed,md,mv,5,sensor);
  delay(1000);
  en.moveLeft(l_speed,r_speed,md,mv,90); //90
  en.moveForward(l_speed,r_speed,md,mv,3,sensor);
  en.wallHugging(l_speed, r_speed, md ,mv ,sensor);
  en.moveForward(l_speed,r_speed,md,mv,5,sensor);
  delay(5000);
  */

  /*
  en.moveForward(l_speed,r_speed,md,mv,3,sensor);
  //en.moveRight(l_speed,r_speed,md,mv,95);
  en.moveLeft(l_speed,r_speed,md,mv,100);
  en.moveForward(l_speed,r_speed,md,mv,3,sensor);
  //en.moveRight(l_speed,r_speed,md,mv,95);
  en.moveLeft(l_speed,r_speed,md,mv,100);
  en.moveForward(l_speed,r_speed,md,mv,3,sensor);
  //en.moveRight(l_speed,r_speed,md,mv,95);
  en.moveLeft(l_speed,r_speed,md,mv,100);  
  en.moveForward(l_speed,r_speed,md,mv,3,sensor);
  //en.moveRight(l_speed,r_speed,md,mv,95);
  en.moveLeft(l_speed,r_speed,md,mv,100);  
  delay(3000);
  */

  /* // Rpi code
    Serial.println("sending to rpi");
    if (Serial.available() > 0) {
    // read the incoming byte:
    String data = Serial.readStringUntil('\n');
    // say what you got:
    Serial.print("I received: ");
    Serial.println(data);
    if (data == "f"){
     md.setSpeeds(300,300); 
    }
  }
  delay(1000);
  */
}
