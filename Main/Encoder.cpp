#include "DualVNH5019MotorShield.h"
#include "Movement.h"
#include "Sensor.h"
#include "Encoder.h"
#include "math.h"

Encoder:: Encoder(int pA1, int pB1, int pA2,  int pB2) {
  //Left
  pinA1 = pA1;
  pinB1 = pB1;
  //Right
  pinA2 = pA2;
  pinB2 = pB2;
}

long unsigned int Encoder::bubbleSort(int numIteration, unsigned long * timeWidth) {
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
  return timeWidth[numIteration / 2];
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
  //fill the array values. can tweak for more accurate readings
  for(int i = 0 ; i < 20; i++){
    ltime[i] = 1832; 
    rtime[i] = 1904; 
  }
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
    rpmTable[0][i / 50 - 1] = bubbleSort(numIteration, timeWidth);
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
    rpmTable[0][i / -50 + 7] = bubbleSort(numIteration, timeWidth);
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
    rpmTable[1][i / 50 - 1] = bubbleSort(numIteration, timeWidth);
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
    rpmTable[1][i / -50 + 7] = bubbleSort(numIteration, timeWidth);
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

void Encoder::ltickIncrement() { //keep 10-20 reading
  unsigned long lcurrent_time = micros();
  // Moving array
  for (int x = 0 ; x < 19; x++) {
    ltime[x] = ltime[x + 1];
  }
  ltime[19] = lcurrent_time - init_ltime;
  //Serial.print("ltime");Serial.println(ltime);
  init_ltime = lcurrent_time;
  ltick++;
}

void Encoder::rtickIncrement() { //keep 10-20 reading
  unsigned long rcurrent_time = micros();
  // Moving array
  for (int x = 0 ; x < 19; x++) {
    rtime[x] = rtime[x + 1];
  }
  rtime[19] = rcurrent_time - init_rtime; // time interval from previous tick
  //Serial.print("rtime");Serial.println(rtime);
  init_rtime = rcurrent_time;
  rtick++;
}

void Encoder::resetTicks() {
  ltick = 0;
  rtick = 0;
  for(int i = 0 ; i < 20; i++){
    ltime[i] = 1832; 
    rtime[i] = 1904; 
  }
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
    md.setSpeeds(i, -i);
    while (tmp_count < numIteration) { // 5 seconds worth
      resetTicks();
      while (ltick < 1) { // wait for tick
      }
      timeWidth[tmp_count] = ltime[19];
      //Serial.println(ltime[19]);
      tmp_count++;
    }
    rpmTable[0][i / 50 - 1] = bubbleSort(numIteration, timeWidth);
    Serial.println(timeWidth[numIteration / 2]);
  }

  //le the motor rest first before computing next rpm
  md.setBrakes(400, 400); // off m1 motor
  delay(1000);


  for (int i = -50; i >= -400; i = i - 50) {
    tmp_count = 0;
    md.setSpeeds(i, -i);
    while (tmp_count < numIteration) { // 5 seconds worth
      resetTicks();
      while (ltick < 1) { // wait for tick
      }
      timeWidth[tmp_count] = ltime[19];
      //Serial.println(ltime[19]);
      tmp_count++;
    }
    rpmTable[0][i / -50 + 7] = bubbleSort(numIteration, timeWidth);
    Serial.println(timeWidth[numIteration / 2]);
  }

  md.setBrakes(400, 400);
  delay(1000);

  // Motor 2
  for (int i = 50; i <= 400; i = i + 50) {
    tmp_count = 0;
    md.setSpeeds(-i, i);
    while (tmp_count < numIteration) { // 5 seconds worth
      resetTicks();
      while (rtick < 1 ) { // wait for tick
      }
      timeWidth[tmp_count] = rtime[19];
      //Serial.println(rtime[19]);
      tmp_count++;
    }
    rpmTable[1][i / 50 - 1] = bubbleSort(numIteration, timeWidth);
    Serial.println(timeWidth[numIteration / 2]);

  }

  //le the motor rest first before computing next rpm
  md.setBrakes(400, 400); // off m1 motor
  delay(1000);


  for (int i = -50; i >= -400; i = i - 50) {
    tmp_count = 0;
    md.setSpeeds(-i, i);
    while (tmp_count < numIteration) { // 5 seconds worth
      resetTicks();
      while (rtick < 1) { // wait for tick
      }
      timeWidth[tmp_count] = rtime[19];
      //Serial.println(rtime[19]);
      tmp_count++;
    }
    rpmTable[1][i / -50 + 7] = bubbleSort(numIteration, timeWidth);
    Serial.println(timeWidth[numIteration / 2]);
  }

  md.setBrakes(400, 400);
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

void Encoder:: printTime() {
  Serial.println(ltime[19]);
}

void Encoder:: stepLTest(DualVNH5019MotorShield md, int timeWidth) {
  md.setM1Speed(-250);
  delay(1000);
  int tmp_count = 0;
  unsigned long start_time = millis();
  while (tmp_count < 100) {
    Serial.println((int)ltime[19]);
    delay(5); // every 0.005 second record one reading
    tmp_count++;
  }
  md.setM1Speed(-300); // change to 250
  while ( ltime[19] >= timeWidth) { // while it havent reach the desired rpm,
    Serial.println(ltime[19]);
    delay(5); // every 0.005 second record one reading
  }
  while (tmp_count < 200) { // stay at 250 for 0.5 second
    Serial.println(ltime[19]);
    delay(5); // every 0.005 second record one reading
    tmp_count++;
  }
  md.setBrakes(400, 400);
}

void Encoder:: stepRTest(DualVNH5019MotorShield md,int timeWidth) {
  md.setM2Speed(250);
  delay(1000);
  int tmp_count = 0;
  while (tmp_count < 100) { // stay at 300 for 0.5 second
    Serial.println(rtime[19]);
    delay(5); // every 0.005 second record one reading
    tmp_count++;
  }
  md.setM2Speed(300);
  while ( rtime[19] >= timeWidth) { // while it havent reach the desired rpm,
    Serial.println(rtime[19]);
    delay(5); // every 0.005 second record one reading
  }
  while (tmp_count < 200) { // stay at 250 for 0.5 second
    Serial.println(rtime[19]);
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

void Encoder::moveForward(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, int gridNum) {
  float distanceTraversed = 0;
  float distanceL = 0;
  float distanceR = 0;
  long tmpl_speed = 0;
  long tmpr_speed = 0;
  //resetDistance();
  rampUp(50,md,mv);
  //md.setSpeeds(setLSpeed, setRSpeed);
  delay(20);
  //delay(2000);
  for ( int i = 0; i < gridNum; i++) {
    resetTicks();
    // because the distance tend to overshoot, it cant be a perfect 10, 9.6 works decent for 3 grids
    // for one grid the braking mechanism cannot stop in time. need take into account decceleration. 7.8 works good for 1 grid
    while (distanceTraversed <= 9.6) { // while havent reach distance, calibrate speed every 0.01seconds
      delay(0.005); // should delay so the speed don't keep changing. need to tweak to get best interval
      tmpl_speed = mv.computeL(setLSpeed, bubbleSort(20,ltime));
      tmpr_speed = mv.computeR(setRSpeed, bubbleSort(20,rtime));
      //Serial.print("tmpl_speed "); Serial.println(tmpl_speed);
      //Serial.print("tmpr_speed "); Serial.println(tmpr_speed);
      md.setSpeeds(tmpl_speed,tmpr_speed);
      distanceL = 2 * 3 * M_PI * (getLticks() / 562.25); 
      distanceR = 2 * 3 * M_PI * (getRticks() / 562.25);
      /*
      Serial.print("distanceL Ticks"); Serial.println(getLticks());
      Serial.print("distanceR Ticks"); Serial.println(getRticks());
      Serial.print("distanceL"); Serial.println(distanceL);
      Serial.print("distanceR"); Serial.println(distanceR);
      */
      distanceTraversed = (distanceL + distanceR) / 2;
    }
    //Serial.print("distanceTraversed"); Serial.println(distanceTraversed);
    distanceTraversed = 0;
    Serial.println("sensor vals"); // make the sensor calibration output a whole number
    // Every 10cm output sensor values
  }
    md.setBrakes(100, 100);
  //resetDistance();

}

void Encoder::moveForwardHug(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, int distance) {
  float distanceTraversed = 0;
  float distanceL = 0;
  float distanceR = 0;
  long tmpl_speed = 0;
  long tmpr_speed = 0;
  //resetDistance();
  //rampUp(setLSpeed,setRSpeed,md);
  md.setSpeeds(setLSpeed, setRSpeed);
  delay(20);
  //delay(2000);
    resetTicks();
    // because the distance tend to overshoot, it cant be a perfect 10, 9.6 works decent for 3 grids
    // for one grid the braking mechanism cannot stop in time. need take into account decceleration. 7.8 works good for 1 grid
    while (distanceTraversed <= distance) { // while havent reach distance, calibrate speed every 0.01seconds
      delay(0.005); // should delay so the speed don't keep changing. need to tweak to get best interval
      tmpl_speed = mv.computeL(setLSpeed, bubbleSort(20,ltime));
      tmpr_speed = mv.computeR(setRSpeed, bubbleSort(20,rtime));
      md.setSpeeds(tmpl_speed,tmpr_speed);
      distanceL = 2 * 3 * M_PI * (getLticks() / 562.25); 
      distanceR = 2 * 3 * M_PI * (getRticks() / 562.25);
      /*
      Serial.print("distanceL Ticks"); Serial.println(getLticks());
      Serial.print("distanceR Ticks"); Serial.println(getRticks());
      Serial.print("distanceL"); Serial.println(distanceL);
      Serial.print("distanceR"); Serial.println(distanceR);
      */
      distanceTraversed = (distanceL + distanceR) / 2;
    }
    md.setBrakes(300, 300);
  //resetDistance();

}

void Encoder::moveLeft(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, float degree) {
  float distanceTraversed = 0;
  float distanceL = 0;
  float distanceR = 0;
  long tmpl_speed = 0;
  long tmpr_speed = 0;
  int initial_delay = 20;
  resetTicks();
  //rampUp(100,100,md);  
  //md.setSpeeds(-setLSpeed, setRSpeed);
  //delay(initial_delay);
  //0.0000004768372
    //while ( distanceTraversed <= (15.27887395921 * M_PI /4 - 0.000000476837186624835)/90 * degree ) { // while havent reach distance, calibrate speed every 0.01seconds
  while ( distanceTraversed <= (15 * M_PI /4)/90 * degree ) {      
      delay(0.005); // should delay so the speed don't keep changing. need to tweak to get best interval
      tmpl_speed = mv.computeL(setLSpeed, bubbleSort(20,ltime));
      tmpr_speed = mv.computeR(setRSpeed, bubbleSort(20,rtime));
      md.setSpeeds(-tmpl_speed,tmpr_speed);
      distanceL = 6 * M_PI * (getLticks() / 562.25); 
      distanceR = 6 * M_PI * (getRticks() / 562.25);
      distanceTraversed = (distanceL + distanceR) / 2;
    }
  md.setBrakes(300, 300);
  //moveForward(setLSpeed, setRSpeed, md, mv, 1);
}

void Encoder::moveRight(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, float degree) {
  float distanceTraversed = 0;
  float distanceL = 0;
  float distanceR = 0;
  int initial_delay = 20;
  resetTicks();
  //rampUp(100,100,md);
  //md.setSpeeds(setLSpeed, -setRSpeed);
  //delay(initial_delay);
   //   while ( distanceTraversed <= (15.27887395921 * M_PI /4 - 0.000000476837186624835)/90 * degree) { // while havent reach distance, calibrate speed every 0.01seconds
   while ( distanceTraversed <= (15 * M_PI /4 )/90 * degree ) {   
      delay(0.005); // should delay so the speed don't keep changing. need to tweak to get best interval
      long tmpl_speed = mv.computeL(setLSpeed, bubbleSort(20,ltime));
      long tmpr_speed = mv.computeR(setRSpeed, bubbleSort(20,rtime));
      md.setSpeeds(tmpl_speed,-tmpr_speed);
      distanceL = 2 * 3 * M_PI * (getLticks() / 562.25); 
      distanceR = 2 * 3 * M_PI * (getRticks() / 562.25);
      distanceTraversed = (distanceL + distanceR) / 2;
    }
  md.setBrakes(300, 300);
  //moveForward(setLSpeed, setRSpeed, md, mv, 1);
}

void Encoder::rampUp(long rpm, DualVNH5019MotorShield md, Movement mv){
 long tmplspeed = mv.convertRSpeed(1);
 long tmprspeed = mv.convertLSpeed(1);
 long tmp_rpm = 0;
  while(tmp_rpm < rpm){
    /*
    if(tmplspeed<lspeed){
      tmplspeed = lspeed;
    }
    if(tmprspeed>rspeed){
      tmprspeed = rspeed;
    }
    */
    tmplspeed = mv.convertRSpeed(tmp_rpm);
    tmprspeed = mv.convertLSpeed(tmp_rpm);
    tmp_rpm++;
    md.setSpeeds(tmplspeed,tmprspeed);
  }
}



void Encoder::moveLeftHug(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv) {
  long distanceTraversed = 0;
  long distanceL = 0;
  long distanceR = 0;
  long tmpl_speed = 0;
  long tmpr_speed = 0;
  int initial_delay = 20;
  resetTicks();
  //0.0000004768372
    while ( distanceTraversed <= 1700 ) { // while havent reach distance, calibrate speed every 0.01seconds
      md.setSpeeds(-setLSpeed,setRSpeed);
      distanceTraversed++;
    }
  md.setBrakes(300, 300);
  //moveForward(setLSpeed, setRSpeed, md, mv, 1);
}

void Encoder::moveRightHug(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv) {
  long distanceTraversed = 0;
  long distanceL = 0;
  long distanceR = 0;
  long tmpl_speed = 0;
  long tmpr_speed = 0;
  int initial_delay = 20;
  resetTicks();
  //0.0000004768372
    while ( distanceTraversed <= 1700 ) { // while havent reach distance, calibrate speed every 0.01seconds
      md.setSpeeds(setLSpeed,-setRSpeed);
      distanceTraversed++;
    }
  md.setBrakes(300, 300);
}

void  Encoder:: wallHugging(long l_speed, long r_speed, DualVNH5019MotorShield md, Movement mv, Sensor sensor){   
    int checked = 0;
    
    //Check if front and back is aligned
    while(checked == 0){
    float sensorA1 = sensor.LFDistance(1);//round(sensor.LFDistance(1)); // left front
    float sensorA2 = sensor.LBDistance(1); //round(sensor.LBDistance(1)); // left back
      Serial.println("calibrating");
      Serial.print("begining sensor left front"); Serial.println(sensorA1);
      Serial.print("begining sensor left back");Serial.println(sensorA2);

      checked = 1; // if entered either of the loops, check again
      while( abs(sensorA1 - sensorA2) >= 0.5){
        if(sensorA1 < sensorA2){ // if sensorA1 is closer to wall than sensorA2      
          moveRightHug(l_speed, r_speed,md,mv);
          delay(300);
          sensorA1 = sensor.LFDistance(1); // left front
          sensorA2 = sensor.LBDistance(1);
          sensorA1 = round(sensorA1);
          sensorA2 = round(sensorA2);
          Serial.print("sensor left front"); Serial.println(sensorA1);
          Serial.print("sensor left back");Serial.println(sensorA2);          
        }
        else{
          moveLeftHug(l_speed, r_speed,md,mv);
          delay(300);
          sensorA1 = (int) sensor.LFDistance(1); // left front
          sensorA2 = (int) sensor.LBDistance(1);
          sensorA1 = round(sensorA1);
          sensorA2 = round(sensorA2);
          Serial.print("sensor left front"); Serial.println(sensorA1);
          Serial.print("sensor left back");Serial.println(sensorA2);          
        }
        checked = 0;
      }
    //check if it is 5 cm
    sensorA1 = round(sensor.LFDistance(1)); // left front
    sensorA2 = round(sensor.LBDistance(1)); // left back
    
      Serial.println("before second loop");
      Serial.print("sensor A1");Serial.println(sensorA1);
      Serial.print("sensor A2");Serial.println(sensorA2);
      if((int) sensorA1 == 5 || (int) sensorA1 == 6 || (int) sensorA1 == 4){
        Serial.println("= 5 || 6 || 4");
      }
      else{
          Serial.println("entered second loop");
          Serial.print("sensor A1");Serial.println(sensorA1);
          Serial.print("sensor A2");Serial.println(sensorA2);
          if(sensorA1 < 5){ // if sensorA1 is closer to wall than sensorA2
            moveRight(l_speed, r_speed,md,mv,90);
            delay(300);
            moveForwardHug(l_speed,r_speed,md,mv,5-sensorA1);
            delay(300);
            moveLeft(l_speed, r_speed,md,mv,90);          
            delay(300);
            sensorA1 = (int) sensor.LFDistance(1); // left front
            sensorA2 = (int) sensor.LBDistance(1);
            sensorA1 = round(sensorA1);
            sensorA2 = round(sensorA2);
          }
          else{
            moveLeft(l_speed, r_speed,md,mv,90);
            delay(300);
            //en.moveForwardHug(l_speed,r_speed,md,mv,sensorA1-5);
            md.setSpeeds(l_speed,r_speed);            
            while( (sensor.FLDistance(1) + sensor.FRDistance(1) )/ 2 > 6){
              Serial.println(sensor.FLDistance(1));
              Serial.println(sensor.FRDistance(1));
              moveForwardHug(l_speed,r_speed,md,mv,0.01);           
            }
            md.setBrakes(100,100);            
            delay(300);
            moveRight(l_speed, r_speed,md,mv,90);
            delay(300);
            sensorA1 = (int) sensor.LFDistance(1); // left front
            sensorA2 = (int) sensor.LBDistance(1);
            sensorA1 = round(sensorA1);
            sensorA2 = round(sensorA2);
          } 
      checked = 0;
      }
  Serial.print("A1 ");Serial.println(sensorA1);
  Serial.print("A2 ");Serial.println(sensorA2);
  }  //md.setM1Brake(-250);
  //exit(1);
}


/*

void  Encoder:: wallHugging(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, Sensor sensor){   
    int checked = 0;
    
    //Check if front and back is aligned
    while(checked == 0){
    float sensorA1 = sensor.LFDistance(1); // left front
    float sensorA2 = sensor.LBDistance(1); // left back
      Serial.println("calibrating");
      Serial.print("begining sensor left front"); Serial.println(sensorA1);
      Serial.print("begining sensor left back");Serial.println(sensorA2);

      checked = 1; // if entered either of the loops, check again
      while((int) sensorA1 != (int) sensorA2){
        if(sensorA1 < sensorA2){ // if sensorA1 is closer to wall than sensorA2      
          en.moveRightHug(l_speed, r_speed,md,mv);
          delay(300);
          sensorA1 = sensor.LFDistance(1); // left front
          sensorA2 = sensor.LBDistance(1);
          sensorA1 = round(sensorA1);
          sensorA2 = round(sensorA2);
          Serial.print("sensor left front"); Serial.println(sensorA1);
          Serial.print("sensor left back");Serial.println(sensorA2);          
        }
        else{
          en.moveLeftHug(l_speed, r_speed,md,mv);
          delay(300);
          sensorA1 = (int) sensor.LFDistance(1); // left front
          sensorA2 = (int) sensor.LBDistance(1);
          sensorA1 = round(sensorA1);
          sensorA2 = round(sensorA2);
          Serial.print("sensor left front"); Serial.println(sensorA1);
          Serial.print("sensor left back");Serial.println(sensorA2);          
        }
        checked = 0;
      }
    //check if it is 5 cm
      Serial.println("before second loop");
      Serial.print("sensor A1");Serial.println(sensorA1);
      Serial.print("sensor A2");Serial.println(sensorA2);
      if((int) sensorA1 == 5 || (int) sensorA1 == 6 || (int) sensorA1 == 4){
        Serial.println("= 5 || 6");
      }
      else{
          Serial.println("entered second loop");
          Serial.print("sensor A1");Serial.println(sensorA1);
          Serial.print("sensor A2");Serial.println(sensorA2);
          if(sensorA1 < 5){ // if sensorA1 is closer to wall than sensorA2
            en.moveRight(l_speed, r_speed,md,mv,90);
            delay(300);
            en.moveForwardHug(l_speed,r_speed,md,mv,5-sensorA1);
            delay(300);
            en.moveLeft(l_speed, r_speed,md,mv,90);          
            delay(300);
            sensorA1 = (int) sensor.LFDistance(1); // left front
            sensorA2 = (int) sensor.LBDistance(1);
            sensorA1 = round(sensorA1);
            sensorA2 = round(sensorA2);
          }
          else{
            en.moveLeft(l_speed, r_speed,md,mv,90);
            delay(300);
            //en.moveForwardHug(l_speed,r_speed,md,mv,sensorA1-5);
            md.setSpeeds(l_speed,r_speed);            
            while( (sensor.FLDistance(1) + sensor.FRDistance(1) )/ 2 > 6){
              Serial.println(sensor.FLDistance(1));
              Serial.println(sensor.FRDistance(1));
              en.moveForwardHug(l_speed,r_speed,md,mv,0.01);           
            }
            md.setBrakes(100,100);            
            delay(300);
            en.moveRight(l_speed, r_speed,md,mv,90);
            delay(300);
            sensorA1 = (int) sensor.LFDistance(1); // left front
            sensorA2 = (int) sensor.LBDistance(1);
            sensorA1 = round(sensorA1);
            sensorA2 = round(sensorA2);
          } 
      checked = 0;
      }
  }  //md.setM1Brake(-250);
  //exit(1);
  delay(1000);
}
 */
