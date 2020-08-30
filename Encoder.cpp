#include "DualVNH5019MotorShield.h"
#include "Movement.h"
#include "Encoder.h"


Encoder:: Encoder(int pA1, int pB1, int pA2,  int pB2) {
  //Left
  pinA1 = pA1;
  pinB1 = pB1;
  //Right
  pinA2 = pA2;
  pinB2 = pB2;
}

void bubbleSort(int numIteration, unsigned long * timeWidth) {
  int out, in, swapper;
  for (out = 0 ; out < numIteration; out++) { // outer loop
    for (in = out; in < (numIteration - 1); in++)  { // inner loop
      if ( timeWidth[in] > timeWidth[in + 1] ) { // out of order?
        // swap them:
        swapper = timeWidth[in];
        timeWidth [in] = timeWidth[in + 1];
        timeWidth[in + 1] = swapper;
      }
    }
  }
}

void Encoder::init() {
  init_ltime = micros();
  init_rtime = init_ltime;
  pinMode(pinA1, INPUT);
  pinMode(pinB1, INPUT);
  pinMode(pinA2, INPUT);
  pinMode(pinB2, INPUT);
  ltick = 0;
  rtick = 0;
  ltime = 0;
  rtime = 0;
}

void Encoder::calibrate(int numIteration, DualVNH5019MotorShield md) {
  unsigned long timeWidth[numIteration] = { 0 };
  md.setM1Speed(0);
  md.setM2Speed(0);
  int tmp_count = 0;

  // Motor 1
  for (int i = 50; i <= 400; i = i + 50) {
    // delay(1000);
    tmp_count = 0;
    md.setM1Speed(i);
    delay(5);
    while (tmp_count < numIteration) { // 5 seconds worth
      int a1_val = pulseIn(pinA1, LOW);
      Serial.print("A1 val: "); Serial.println(a1_val);
      timeWidth[tmp_count] = a1_val;
      tmp_count++;
    }
    bubbleSort(numIteration, timeWidth);
    rpmTable[0][i / 50 - 1] = timeWidth[numIteration / 2];
  }

  //le the motor rest first before computing next rpm
  md.setM1Brake(400); // off m1 motor
  delay(1000);


  for (int i = -50; i >= -400; i = i - 50) {
    // delay(1000);
    tmp_count = 0;
    md.setM1Speed(i);
    delay(5);
    while (tmp_count < numIteration) { // 5 seconds worth
      int a1_val = pulseIn(pinA1, LOW);
      Serial.print("A1 val: "); Serial.println(a1_val);
      timeWidth[tmp_count] = a1_val;
      tmp_count++;
    }
    bubbleSort(numIteration, timeWidth);
    rpmTable[0][i / -50 + 7] = timeWidth[numIteration / 2];
  }

  md.setM1Brake(400); // off m1 motor
  delay(1000);

  //Motor 2
  for (int i = 50; i <= 400; i = i + 50) {
    tmp_count = 0;
    md.setM2Speed(i);
    delay(5);
    while (tmp_count < numIteration) { // 5 seconds worth
      int a2_val = pulseIn(pinA2, LOW);
      Serial.print("A2 val: "); Serial.println(a2_val);
      timeWidth[tmp_count] = a2_val;
      tmp_count++;
    }
    bubbleSort(numIteration, timeWidth);
    rpmTable[1][i / 50 - 1] = timeWidth[numIteration / 2];
  }
  md.setM2Brake(400);
  delay(1000);
  for (int i = -50; i >= -400; i = i - 50) {
    // delay(1000);
    tmp_count = 0;
    md.setM2Speed(i);
    delay(5);
    while (tmp_count < numIteration) { // 5 seconds worth
      int a2_val = pulseIn(pinA2, LOW);
      Serial.print("A2 val: "); Serial.println(a2_val);
      timeWidth[tmp_count] = a2_val;
      tmp_count++;
    }
    bubbleSort(numIteration, timeWidth);
    rpmTable[1][i / -50 + 7] = timeWidth[numIteration / 2];
  }

  md.setBrakes(400, 400);
  /*
    for(int i =0 ;i < 8; i++){
    Serial.print("A1 Val for rpm ");Serial.print((i+1)*50); Serial.print(": "); Serial.println(rpmTable[0][i]);
    }
  */
  for (int i = 0 ; i < 8; i++) {
    Serial.print("A1 Val for rpm "); Serial.print((i + 1) * -50); Serial.print(": "); Serial.println(rpmTable[0][i + 8]);
  }
  for (int i = 0 ; i < 8; i++) {
    Serial.print("A2 Val for rpm "); Serial.print((i + 1) * 50); Serial.print(": "); Serial.println(rpmTable[1][i]);
  }
  /*
    for(int i =0 ;i < 8; i++){
    Serial.print("A2 Val for rpm ");Serial.print((i+1)* -50); Serial.print(": "); Serial.println(rpmTable[1][i+8]);
    }
  */
  delay(20000);
}

void Encoder::ltickIncrement() {
  unsigned long lcurrent_time = micros();
  ltime = lcurrent_time - init_ltime; // time interval from previous tick
  //Serial.print("ltime");Serial.println(ltime);
  init_ltime = lcurrent_time;
  ltick++;
}

void Encoder::rtickIncrement() {
  unsigned long rcurrent_time = micros();
  rtime = rcurrent_time - init_rtime; // time interval from previous tick
  //Serial.print("rtime");Serial.println(rtime);
  init_rtime = rcurrent_time;
  rtick++;
}

void Encoder::resetTicks() {
  ltick = 0;
  rtick = 0;
  ltime = 0;
  rtime = 0;
}

int Encoder::getLticks() {
  return ltick;
}

int Encoder::getRticks() {
  return rtick;
}

void Encoder::tickCal(int numIteration, DualVNH5019MotorShield md) {
  unsigned long timeWidth[numIteration] = { 0 }; //lazy to change the name
  int tmp_count = 0;
  // Motor 1
  for (int i = 50; i <= 400; i = i + 50) {
    tmp_count = 0;
    md.setSpeeds(i,-i);
    while (tmp_count < numIteration) { // 5 seconds worth
      resetTicks();
      while (ltick < 1) { // wait for tick
      }
      timeWidth[tmp_count] = ltime;
      tmp_count++;
    }
    bubbleSort(numIteration, timeWidth);
    rpmTable[0][i / 50 - 1] = timeWidth[numIteration / 2];
    //Serial.println(timeWidth[numIteration/2]);
  }

  //le the motor rest first before computing next rpm
  md.setBrakes(400,400); // off m1 motor
  delay(1000);


  for (int i = -50; i >= -400; i = i - 50) {
    tmp_count = 0;
    md.setSpeeds(i,-i);
    while (tmp_count < numIteration) { // 5 seconds worth
      resetTicks();
      while (ltick < 1) { // wait for tick
      }
      timeWidth[tmp_count] = ltime;
      tmp_count++;
    }
    bubbleSort(numIteration, timeWidth);
    rpmTable[0][i / -50 + 7] = timeWidth[numIteration / 2];
    //Serial.println(timeWidth[numIteration/2]);
  }

  md.setBrakes(400,400);
  delay(1000);

  // Motor 2
  for (int i = 50; i <= 400; i = i + 50) {
    tmp_count = 0;
    md.setSpeeds(-i,i);
    while (tmp_count < numIteration) { // 5 seconds worth
      resetTicks();
      while (rtick < 1 ) { // wait for tick
      }
      timeWidth[tmp_count] = rtime;
      tmp_count++;
    }
    bubbleSort(numIteration, timeWidth);
    rpmTable[1][i / 50 - 1] = timeWidth[numIteration / 2];
    //Serial.println(timeWidth[numIteration/2]);

  }

  //le the motor rest first before computing next rpm
  md.setBrakes(400,400); // off m1 motor
  delay(1000);


  for (int i = -50; i >= -400; i = i - 50) {
    tmp_count = 0;
    md.setSpeeds(-i,i);
    while (tmp_count < numIteration) { // 5 seconds worth
      resetTicks();
      while (rtick < 1) { // wait for tick
      }
      timeWidth[tmp_count] = rtime;
      tmp_count++;
    }
    bubbleSort(numIteration, timeWidth);
    rpmTable[1][i / -50 + 7] = timeWidth[numIteration / 2];
    //Serial.println(timeWidth[numIteration/2]);
  }

  md.setBrakes(400,400);
  delay(1000);

  for (int i = 0 ; i < 8; i++) {
    Serial.print("A1 Val for rpm "); Serial.print((i + 1) * 50); Serial.print(": "); Serial.println(rpmTable[0][i]);
  }
  for (int i = 0 ; i < 8; i++) {
    Serial.print("A1 Val for rpm "); Serial.print((i + 1) * -50); Serial.print(": "); Serial.println(rpmTable[0][i + 8]);
  }
  for (int i = 0 ; i < 8; i++) {
    Serial.print("A2 Val for rpm "); Serial.print((i + 1) * 50); Serial.print(": "); Serial.println(rpmTable[1][i]);
  }
  for (int i = 0 ; i < 8; i++) {
    Serial.print("A2 Val for rpm "); Serial.print((i + 1) * -50); Serial.print(": "); Serial.println(rpmTable[1][i + 8]);
  }

}

void Encoder::calcRPM(unsigned long start_time) {
  unsigned long stop_time = micros();
  float rpm = (ltick / 562.25) / ((stop_time - start_time) / 60000);
  Serial.print("left motor: "); Serial.print(rpm); Serial.print("lticks"); Serial.println(ltick);
  rpm = (rtick / 562.25) / ((stop_time - start_time) / 60000);
  Serial.print("right motor: "); Serial.print(rpm); Serial.print("rticks"); Serial.println(rtick);
}

void Encoder:: stepLTest(DualVNH5019MotorShield md) {
  md.setM1Speed(-250);
  delay(1000);
  int tmp_count = 0;
  unsigned long start_time = millis();
  while (tmp_count < 100) {
    Serial.println(ltime);
    delay(5); // every 0.005 second record one reading
    tmp_count++;
  }
  md.setM1Speed(-300); // change to 250
  while ( ltime >= 1028) { // while it havent reach the desired rpm,
    Serial.println(ltime);
    delay(5); // every 0.005 second record one reading
  }
  while (tmp_count < 200) { // stay at 250 for 0.5 second
    Serial.println(ltime);
    delay(5); // every 0.005 second record one reading
    tmp_count++;
  }
  md.setBrakes(400, 400);
}

void Encoder:: stepRTest(DualVNH5019MotorShield md) {
  md.setM2Speed(250);
  delay(1000);
  int tmp_count = 0;
  while (tmp_count < 100) { // stay at 300 for 0.5 second
    Serial.println(rtime);
    delay(5); // every 0.005 second record one reading
    tmp_count++;
  }
  md.setM2Speed(300);
  while ( rtime >= 1052) { // while it havent reach the desired rpm,
    Serial.println(rtime);
    delay(5); // every 0.005 second record one reading
  }
  while (tmp_count < 200) { // stay at 250 for 0.5 second
    Serial.println(rtime);
    delay(5); // every 0.005 second record one reading
    tmp_count++;
  }
  md.setBrakes(400, 400);
}


void Encoder:: stepPLTest(DualVNH5019MotorShield md) {
  md.setM1Speed(-250); // change to 250
  delay(1000);
  //unsigned long time = millis();
  int tmp_count = 0;
  while (tmp_count < 10) {
    Serial.println(pulseIn(pinA1, LOW));
    delay(5); // every 0.005 second record one reading
    tmp_count++;
  }
  unsigned long pulse = pulseIn(pinA1, LOW);
  Serial.println(pulse);
  md.setM1Speed(-300); // change to 250
  while ( pulse >= 542) { //542 // while it havent reach the desired rpm,
    delay(5); // every 0.005 second record one reading
    pulse = pulseIn(pinA1, LOW);
    Serial.println(pulse);
  }
  while (tmp_count < 20) {
    Serial.println(pulseIn(pinA1, LOW));
    delay(5); // every 0.005 second record one reading
    tmp_count++;
  }
  md.setBrakes(400, 400);
}
/*
  void Encoder:: stepPRTest(DualVNH5019MotorShield md){
  md.setSpeeds(300,300);
  delay(1000);
  unsigned long time = millis();
  while(millis()-time < 500){ // stay at 300 for 0.5 second
    delay(5); // every 0.005 second record one reading
    Serial.println(pulseIn(pinA2,LOW));
  }
  md.setSpeeds(250,250); // change to 250
  while( pulseIn(pinA2,LOW) <= 769 ){ // while it havent reach the desired rpm,
    delay(5); // every 0.005 second record one reading
    Serial.println(pulseIn(pinA2,LOW));
  }
  unsigned long end_time = millis();
  while(millis()-end_time < 500){ // stay at 250 for 0.5 second
    delay(5); // every 0.005 second record one reading
    Serial.println(pulseIn(pinA2,LOW));
  }
  md.setBrakes(400,400);
  }
*/

void Encoder:: stepPRTest(DualVNH5019MotorShield md) {
  md.setM2Speed(250); // change to 250
  delay(1000);
  int tmp_count = 0;
  while (tmp_count < 10) {
    Serial.println(pulseIn(pinA2, LOW));
    delay(5); // every 0.005 second record one reading
    tmp_count++;
  }
  unsigned long pulse = pulseIn(pinA2, LOW);
  Serial.println(pulse);
  md.setM2Speed(300); // change to 250
  while ( pulse >= 620 ) { //620 // while it havent reach the desired rpm,
    pulse = pulseIn(pinA2, LOW);
    Serial.println(pulse);
    delay(5); // every 0.005 second record one reading
  }
  tmp_count = 0;
  while (tmp_count < 20) {
    Serial.println(pulseIn(pinA2, LOW));
    delay(5); // every 0.005 second record one reading
    tmp_count++;
  }
  md.setBrakes(400, 400);
}

void Encoder::moveForward(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv){
  long distanceTraversed = 0;
  long distanceL = 0;
  long distanceR = 0;
  //resetDistance();
  md.setSpeeds(setLSpeed,setRSpeed);
  delay(10);
  //delay(2000);
  while(distanceTraversed <= 100){ // while havent reach distance, calibrate speed every 0.01seconds
    delay(0.005); // should delay so the speed don't keep changing. need to tweak to get best interval
    //Serial.println(pulseIn(pinA1,LOW));
    long tmpl_speed = mv.computeL(setLSpeed,(pulseIn(pinA1,LOW) * 2 ));
    long tmpr_speed = mv.computeR(setRSpeed,(pulseIn(pinA2,LOW) * 2 ));
    Serial.print("lspeed");Serial.println(tmpl_speed);
    Serial.print("rspeed");Serial.println(tmpr_speed);
    md.setM1Speed(tmpl_speed);
    md.setM2Speed(tmpr_speed);
    distanceL += 6 * (getLticks()/562.25); //assuming wheelradius = 6
    distanceR += 6 * (getRticks()/562.25);
    //Serial.println(distanceL);
    //Serial.println(distanceR);
    resetTicks();
    distanceTraversed = (distanceL + distanceR)/2;
    //Serial.println(distanceTraversed);
  }
 
//resetDistance();
}
