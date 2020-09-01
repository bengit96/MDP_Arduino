#include <PID_v1.h>
#include "DualVNH5019MotorShield.h"
#include "math.h"

//Encoder pin
#define enA1 3 // Left
#define enB1 5 // Left
#define enA2 11 // Right
#define enB2 13 // Right

//Define time
double LTime, RTime;

//Define speeds
double LSpeed, RSpeed;

//Define Variables we'll be connecting to
double Setpoint, LInput, LOutput, RInput, ROutput;

//Define the aggressive and conservative Tuning Parameters [LEFT]
double aggLKp=4, aggLKi=0.2, aggLKd=1;
double consLKp=1.799735079, consLKi=213.3143124, consLKd=0.34743448964;

//Define the aggressive and conservative Tuning Parameters [RIGHT]
double aggRKp=4, aggRKi=0.2, aggRKd=1;
double consRKp=3.391186, consRKi=71.243, consRKd=0.0403551241;

//Specify the links and initial tuning parameters
PID myLPID(&LInput, &LOutput, &Setpoint, consLKp, consLKi, consLKd, DIRECT);
PID myRPID(&RInput, &ROutput, &Setpoint, consRKp, consRKi, consRKd, DIRECT);

DualVNH5019MotorShield md;

void stopIfFault() {
  if (md.getM1Fault()) {
    Serial.println("L Motor Fault");
    while(1);
  }
  if (md.getM2Fault()) {
    Serial.println("R Motor Fault");
    while(1);
  }
}

void setup()
{
  Serial.begin(19200);
  //initialize the variables we're linked to
  Setpoint = 50; //RPM that we want
  LSpeed = (-2.7814)*Setpoint - 21.8053;
  RSpeed = 2.8291*Setpoint + 17.4258;

  //initialise motor
  md.init();
  md.setSpeeds(LSpeed,RSpeed);
  stopIfFault();
  delay(1000);
  
  //turn the PID on
  myLPID.SetMode(AUTOMATIC);
  myRPID.SetMode(AUTOMATIC);
}

void loop()
{
  //md.setSpeeds(200,200);
 
  LTime = pulseIn(enA1, HIGH);
  RTime = pulseIn(enA2, HIGH);
  Serial.print("LTime: ");Serial.println(LTime);
  Serial.print("RTime: ");Serial.println(LTime);
  RInput = ((1/(LTime*2)) * pow(10,6) * 60) / 562.5; //Supposed Left
  LInput = ((1/(RTime*2)) * pow(10,6) * 60) / 562.5; //Supposed Right
  Serial.print("LInput: ");Serial.println(LInput);
  Serial.print("RInput: ");Serial.println(RInput);
  double Lgap = abs(Setpoint-LInput); //distance away from setpoint
  double Rgap = abs(Setpoint-RInput); 

  /*
  if(Lgap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }*/

  
  myLPID.Compute(); myRPID.Compute();
  Serial.print("LOutput: ");Serial.println(LOutput);
  Serial.print("ROutput: ");Serial.println(ROutput);
  LSpeed = (-2.7814)*LOutput - 21.8053;
  RSpeed = 2.8291*ROutput + 17.4258;
  if (RSpeed > 400) {
    RSpeed = 300;
  }
  if (LSpeed < -400) {
    LSpeed = -300;
  }
  Serial.print("LSpeed: ");Serial.println(LSpeed);
  Serial.print("RSpeed: ");Serial.println(RSpeed);
  md.setSpeeds(LSpeed,RSpeed);
  stopIfFault();
  delay(1000);
}
