#include "EnableInterrupt.h"
#include "DualVNH5019MotorShield.h"
#include "Movement.h"
#include "Encoder.h"

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

float lkP = 2.905382162; //1.452691081;
float lkI = 0;//-216.3143124;//0.9316770186;
float lkD = 0;//-0.3743448964;//0.001612322981;

float rkP = 8.646;//2.580982188; //1.290491094;
float rkI = 0;//352.9411765;//0.03529411765;
float rkD = 0;//0.03529411765;//0.000003529411765;



// Declaration
DualVNH5019MotorShield md;
Encoder en(enA1,enB1,enA2,enB2);
Movement mv(enA2,enB1,enA2,enB2,lkP,lkI,lkD,rkP,rkI,rkD);


static volatile unsigned long ltime_tmp = 0; // volatile will tell the compiler that the value must be checked every time
static volatile unsigned long rtime_tmp = 0;  


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
  Serial.println("TEST");
  enableInterrupt(en.pinA1, leftmotor, RISING);
  enableInterrupt(en.pinA2, rightmotor, RISING);
  //Serial.print(lkP,4);
  //delay(10000);
  //en.calibrate(300, md); //Calibrate table lookup
  //en.tickCal(300,md);
  //en.stepTest(md);
  
  /*
  Serial.println("left");
  en.stepLTest(md);
  Serial.println("right");
  en.stepRTest(md);
  */
  /*
  Serial.println("left"); 
  en.stepPLTest(md);// . not sure if Step test for L will be an issue since its negative speed (i.e -250 to -300)
  Serial.println("right");
  en.stepPRTest(md);
  */
  
  float rpm = 50; // change accordingly
  l_speed = mv.convertLSpeed(rpm); // change this functions based on gradient found and y intercept
  r_speed = mv.convertRSpeed(rpm); // change this functions based on gradient found and y intercept
  Serial.print("l_speed");Serial.println(l_speed); // put the speeds into moveForward function
  Serial.print("r_speed");Serial.println(r_speed);
  
}


void loop() {
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
  // md.setSpeeds(l_speed,r_speed);
  en.moveForward(l_speed, r_speed,md,mv);// r and l speed need to change to fixed values
  //delay(1000);
  //Serial.println("sending to rpi");
  //delay(1000);
  /*
    md.setSpeeds(-390,400);
    en.resetTicks();
    unsigned long start_time = micros();
    delay(5000);
    md.setBrakes(400,400);
    delay(1000);
    en.calcRPM(start_time);
    */
  //Serial.println("loop");
  // unsigned long time = micros();
  // Serial.println(time);
	// Serial.println("Entered loop");
  //double sensorVal = sensorCal(ps1);
  // Serial.println(sensorVal);
  /*
  md.setM1Speed(m1_speed);
  md.setM2Speed(m2_speed);
  long a1_val = pulseIn(enA1,LOW);
  long a2_val = pulseIn(enA2,LOW);
  long b1_val = pulseIn(enB1,LOW);
  long b2_val = pulseIn(enB2,LOW);

  Serial.println("Motor 1");
  Serial.print("A1 val: "); Serial.println(a1_val);
  Serial.print("b1_val: "); Serial.println(b1_val);
  delay(300);
  Serial.println("Motor 2");
  Serial.print("A2 val: "); Serial.println(a2_val);
  Serial.print("b2_val: "); Serial.println(b2_val);
  */
}
