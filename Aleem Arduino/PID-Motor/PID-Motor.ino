#include <PID_v1.h>
#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

char receivedChar;
boolean newData = false;

unsigned long changeTime = 0; //last time the sensor was triggered
volatile unsigned long quarterSpins = 0;
unsigned long currentTime = millis();
double driverOut = 220;
double difference = 10;
double setPoint = 10;
String inString;
long Kp = 0.007578947368;
long Ki = 0.09473684211;
long Kd = 0.000151578947;

PID myPID(&difference, &driverOut, &setPoint,Kp,Ki,Kd, DIRECT);
PID myPID(&difference, &driverM2Out, &setPoint,Kp2,Ki2,Kd2, DIRECT);

void setup() {

  myPID.SetMode(AUTOMATIC);
  md.init();
  
  attachInterrupt(digitalPinToInterrupt(2), blink, CHANGE);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  digitalWrite(3, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.begin(9600);

}

void loop() {
  myPID.Compute();
  md.setM1Speed(driverOut);
  Serial.print("  difference: "); Serial.print(difference); Serial.print("  ");
  Serial.print("setPoint: "); Serial.print(setPoint); Serial.print("  ");
  Serial.print("driverOut: "); Serial.println(driverOut);
  digitalWrite(LED_BUILTIN, LOW);
  recvOneChar();
  showNewData();
}

void blink() {
  quarterSpins++;
  currentTime = millis();
  difference = currentTime - changeTime;
  changeTime = currentTime;
  
  digitalWrite(LED_BUILTIN, HIGH);
}

void recvOneChar() {
   if (Serial.available() > 0) {
     receivedChar = Serial.read();
     newData = true;
   }
}

void showNewData() {
  if (newData == true) {
    if (isDigit(receivedChar)) {
      inString += (char)receivedChar;}
    if (receivedChar == '\n') {
      setPoint = (inString.toInt());
      inString = "";
      newData = false;
      }
  }
}
