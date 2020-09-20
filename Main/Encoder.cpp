#include "DualVNH5019MotorShield.h"
#include "Movement.h"
#include "Sensor.h"
#include "Encoder.h"
#include "math.h"

int rpmTable[2][16];


Encoder:: Encoder(int pA1, int pB1, int pA2,  int pB2) {
  //Left
  pinA1 = pA1;
  pinB1 = pB1;
  //Right
  pinA2 = pA2;
  pinB2 = pB2;
}

long unsigned int Encoder::insertionSort(int numIteration, unsigned long * timeWidth) {
  int out, in, swapper;
  for (out = 1 ; out < numIteration; out++) { // outer loop
    for (in = out; in > 0; in--)  { // inner loop
      if ( timeWidth[in] < timeWidth[in - 1] ) { //Each iteration compare till larger than previous element
        // swap them:
        swapper = timeWidth[in];
        timeWidth [in] = timeWidth[in - 1];
        timeWidth[in - 1] = swapper;
      }
      else 
        break;
    }
  }
  return timeWidth[numIteration / 2];
}

/*
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
*/

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
  for (int i = 0 ; i < 9; i++) {
    ltime[i] = 1832;
    rtime[i] = 1904;
  }
}

void Encoder::ltickIncrement() { //keep 10-20 reading
  unsigned long lcurrent_time = micros();
  // Moving array
  for (int x = 0 ; x < 9; x++) {
    ltime[x] = ltime[x + 1];
  }
  ltime[9] = lcurrent_time - init_ltime;
  init_ltime = lcurrent_time;
  ltick++;
}

void Encoder::rtickIncrement() { //keep 10-20 reading
  unsigned long rcurrent_time = micros();
  // Moving array
  for (int x = 0 ; x < 9; x++) {
    rtime[x] = rtime[x + 1];
  }
  rtime[9] = rcurrent_time - init_rtime; // time interval from previous tick
  init_rtime = rcurrent_time;
  rtick++;
}

void Encoder::resetTicks() {
  ltick = 0;
  rtick = 0;
  for (int i = 0 ; i < 9; i++) {
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

  for (int i = -50; i >= -400; i = i - 50) {
    tmp_count = 0;
    md.setSpeeds(i, -i);
    while (tmp_count < numIteration) { // 5 seconds worth
      resetTicks();
      while (ltick < 1) { // wait for tick
      }
      timeWidth[tmp_count] = ltime[9];
      //Serial.println(ltime[19]);
      tmp_count++;
    }
    rpmTable[0][i / -50 + 7] = insertionSort(numIteration, timeWidth);
    Serial.println(timeWidth[numIteration / 2]);
  }

  md.setBrakes(400, 400);
  delay(2000);

  // Motor 2
  for (int i = 50; i <= 400; i = i + 50) {
    tmp_count = 0;
    md.setSpeeds(-i, i);
    while (tmp_count < numIteration) { // 5 seconds worth
      resetTicks();
      while (rtick < 1 ) { // wait for tick
      }
      timeWidth[tmp_count] = rtime[9];
      //Serial.println(rtime[19]);
      tmp_count++;
    }
    rpmTable[1][i / 50 - 1] = insertionSort(numIteration, timeWidth);
    Serial.println(timeWidth[numIteration / 2]);

  }
  md.setBrakes(300,300);
  delay(2000);
  //Step test
  stepLTest(md,rpmTable[0][13]);
  stepRTest(md,rpmTable[1][5]);

}


void Encoder:: stepLTest(DualVNH5019MotorShield md, int timeWidth) {
  md.setSpeeds(-250, 250);
  delay(1000);
  int tmp_count = 0;
  unsigned long start_time = millis();
  while (tmp_count < 100) {
    Serial.println((int)ltime[9]);
    delay(5); // every 0.005 second record one reading
    tmp_count++;
  }
  md.setSpeeds(-300, 300); // change to 250
  //md.setSpeeds(-350, 350); // change to 250
  while ( ltime[9] >= timeWidth) { // while it havent reach the desired rpm,
    Serial.println(ltime[9]);
    delay(5); // every 0.005 second record one reading
  }
  while (tmp_count < 200) { // stay at 250 for 0.5 second
    Serial.println(ltime[9]);
    delay(5); // every 0.005 second record one reading
    tmp_count++;
  }
  md.setBrakes(400, 400);
}

void Encoder:: stepRTest(DualVNH5019MotorShield md, int timeWidth) {
  md.setSpeeds(-250, 250);
  delay(1000);
  int tmp_count = 0;
  while (tmp_count < 100) { // stay at 300 for 0.5 second
    Serial.println(rtime[9]);
    delay(5); // every 0.005 second record one reading
    tmp_count++;
  }
  md.setSpeeds(-300, 300);
  while ( rtime[9] >= timeWidth) { // while it havent reach the desired rpm,
    Serial.println(rtime[9]);
    delay(5); // every 0.005 second record one reading
  }
  while (tmp_count < 200) { // stay at 250 for 0.5 second
    Serial.println(rtime[9]);
    delay(5); // every 0.005 second record one reading
    tmp_count++;
  }
  md.setBrakes(400, 400);
}


void Encoder::moveForward(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, int gridNum, Sensor sensor) {
  float distanceTraversed = 0;
  float distanceL = 0;
  float distanceR = 0;
  long tmpl_speed = 0;
  long tmpr_speed = 0;
  float setDistance = 9.6;
  //resetDistance();
  //rampUp(50, md, mv); //remember to change this when change rpm
  //md.setSpeeds(setLSpeed, setRSpeed);
  
  //delay(2000);
  for ( int i = 0; i < gridNum; i++) {
    resetTicks();
    // because the distance tend to overshoot, it cant be a perfect 10, 9.6 works decent for 3 grids
    // for one grid the braking mechanism cannot stop in time. need take into account decceleration. 7.8 works good for 1 grid
    if ( i + 1 == gridNum) { //for last grid, setdistance smaller to accomodate braking
      setDistance = 6;
    }
    //rampUp(50, md, mv); //remember to change this when change rpm
    //delay(500);
    while (distanceTraversed <= setDistance) { // while havent reach distance, calibrate speed every 0.01seconds
      delay(50); // should delay so the speed don't keep changing. need to tweak to get best interval
      //delay(100); // for kp ki kd
      tmpl_speed = mv.computeL(setLSpeed, insertionSort(10, ltime));
      tmpr_speed = mv.computeR(setRSpeed, insertionSort(10, rtime));
      //Serial.println(tmpl_speed);
      //Serial.println(tmpr_speed);
      md.setSpeeds(tmpl_speed, tmpr_speed);
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
  
    int flgrid = sensor.convertShort(sensor.FLDistance(1));
    int fmgrid = sensor.convertShort(sensor.FMDistance(1));
    int frgrid = sensor.convertShort(sensor.FRDistance(1));
    int lfgrid = sensor.convertShort(sensor.LFDistance(1));
    int lbgrid = sensor.convertShort(sensor.LBDistance(1));
    
    Serial.print(sensor.LBDistance(1)); Serial.print(" ");
    Serial.print(sensor.LFDistance(1)); Serial.print(" ");
    Serial.print(sensor.FLDistance(1)); Serial.print(" ");
    Serial.print(sensor.FMDistance(1)); Serial.print(" ");
    Serial.print(sensor.FRDistance(1)); Serial.print(" ");
    float rdistance = sensor.RDistance(1);
    int rgrid = sensor.convertLong(rdistance);      
    Serial.println(rdistance); Serial.print(" ");
     
    Serial.print(lbgrid); Serial.print(" ");
    Serial.print(lfgrid); Serial.print(" ");
    Serial.print(flgrid); Serial.print(" ");
    Serial.print(fmgrid); Serial.print(" ");
    Serial.print(frgrid); Serial.print(" ");     
    Serial.println(rgrid); Serial.print(" ");
    /*
      Serial.print("front left : ");Serial.println(flgrid);
      Serial.print("front middle : ");Serial.println(fmgrid);
      Serial.print("front right : ");Serial.println(frgrid);
      Serial.print("left front : ");Serial.println(lfgrid);
      Serial.print("left back: ");Serial.println(lbgrid);
      Serial.print("right : ");Serial.println(rgrid);
    */
    // Every 10cm output sensor values
  
  }
  rampDown(100, md, mv); //remember to change this when change rpm
  md.setBrakes(300, 300);
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
    tmpl_speed = mv.computeL(setLSpeed, insertionSort(10, ltime));
    tmpr_speed = mv.computeR(setRSpeed, insertionSort(10, rtime));
    md.setSpeeds(tmpl_speed, tmpr_speed);
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
  float rpm_cali = 100;
  long lc_speed = mv.convertLSpeed(rpm_cali);
  long rc_speed = mv.convertRSpeed(rpm_cali);

  resetTicks();
  //rampUp(100,100,md);
  //md.setSpeeds(-setLSpeed, setRSpeed);
  //delay(initial_delay);
  //0.0000004768372
  //while ( distanceTraversed <= (15.27887395921 * M_PI /4 - 0.000000476837186624835)/90 * degree ) { // while havent reach distance, calibrate speed every 0.01seconds
  while ( distanceTraversed <= (15 * M_PI / 4) / 90 * degree ) {
    delay(0.005); // should delay so the speed don't keep changing. need to tweak to get best interval
    tmpl_speed = mv.computeL(lc_speed, insertionSort(10, ltime));
    tmpr_speed = mv.computeR(rc_speed, insertionSort(10, rtime));
    md.setSpeeds(-tmpl_speed, tmpr_speed);
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
  float rpm_cali = 100;
  long lc_speed = mv.convertLSpeed(rpm_cali);
  long rc_speed = mv.convertRSpeed(rpm_cali);
  delay(500);
  resetTicks();
  //rampUp(100,100,md);
  //md.setSpeeds(setLSpeed, -setRSpeed);
  //delay(initial_delay);
  //   while ( distanceTraversed <= (15.27887395921 * M_PI /4 - 0.000000476837186624835)/90 * degree) { // while havent reach distance, calibrate speed every 0.01seconds
  while ( distanceTraversed <= (15 * M_PI / 4 ) / 90 * degree ) {
    delay(0.005); // should delay so the speed don't keep changing. need to tweak to get best interval
    long tmpl_speed = mv.computeL(lc_speed, insertionSort(10, ltime));
    long tmpr_speed = mv.computeR(rc_speed, insertionSort(10, rtime));
    md.setSpeeds(tmpl_speed, -tmpr_speed);
    distanceL = 2 * 3 * M_PI * (getLticks() / 562.25);
    distanceR = 2 * 3 * M_PI * (getRticks() / 562.25);
    distanceTraversed = (distanceL + distanceR) / 2;
  }
  md.setBrakes(300, 300);
  //moveForward(setLSpeed, setRSpeed, md, mv, 1);
}

void Encoder::rampUp(long rpm, DualVNH5019MotorShield md, Movement mv) {
  long tmplspeed = mv.convertRSpeed(1);
  long tmprspeed = mv.convertLSpeed(1);
  long tmp_rpm = 0;
  while (tmp_rpm < rpm) {
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
    md.setSpeeds(-tmplspeed, -tmprspeed);
  }
}

void Encoder::rampDown(long rpm, DualVNH5019MotorShield md, Movement mv) {
  long tmplspeed = mv.convertRSpeed(1);
  long tmprspeed = mv.convertLSpeed(1);
  //long tmp_rpm = 0;
  while (rpm != 0) {
    /*
      if(tmplspeed<lspeed){
      tmplspeed = lspeed;
      }
      if(tmprspeed>rspeed){
      tmprspeed = rspeed;
      }
    */
    tmplspeed = mv.convertRSpeed(rpm);
    tmprspeed = mv.convertLSpeed(rpm);
    rpm -= 2;
    md.setSpeeds(tmplspeed, tmprspeed);
  }
  md.setBrakes(300, 300);
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
    md.setSpeeds(-setLSpeed, setRSpeed);
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
    md.setSpeeds(setLSpeed, -setRSpeed);
    distanceTraversed++;
  }
  md.setBrakes(300, 300);
}

void  Encoder:: wallHugging(long l_speed, long r_speed, DualVNH5019MotorShield md, Movement mv, Sensor sensor) {
  int checked = 0;

  float rpm_cali = 20;
  long lc_speed = mv.convertLSpeed(rpm_cali);
  long rc_speed = mv.convertRSpeed(rpm_cali);

  //Check if front and back is aligned
  while (checked == 0) {
    float sensorA1 = sensor.LFDistance(1);//round(sensor.LFDistance(1)); // left front
    float sensorA2 = sensor.LBDistance(1); //round(sensor.LBDistance(1)); // left back

    checked = 1; // if entered either of the loops, check again
    while ( abs(sensorA1 - sensorA2) >= 0.05) {
      if (sensorA1 < sensorA2) { // if sensorA1 is closer to wall than sensorA2
        moveRightHug(lc_speed, rc_speed, md, mv);
        //delay(10);
        sensorA1 = sensor.LFDistance(1); // left front
        sensorA2 = sensor.LBDistance(1);
        sensorA1 = round(sensorA1);
        sensorA2 = round(sensorA2);
        //Serial.print("sensor left front: "); Serial.println(sensorA1);
        //Serial.print("sensor left back: "); Serial.println(sensorA2);
      }
      else {
        moveLeftHug(lc_speed, rc_speed, md, mv);
        //delay(10);
        sensorA1 = (int) sensor.LFDistance(1); // left front
        sensorA2 = (int) sensor.LBDistance(1);
        sensorA1 = round(sensorA1);
        sensorA2 = round(sensorA2);
        //Serial.print("sensor left front: "); Serial.println(sensorA1);
        //Serial.print("sensor left back: "); Serial.println(sensorA2);
      }
      checked = 0;
    }
        Serial.print("sensor left front: "); Serial.println(sensorA1);
        Serial.print("sensor left back: "); Serial.println(sensorA2);

    if ((int) sensorA1 == 5 || (int)sensorA1 == 6 || (int)  sensorA1 == 7) {
      Serial.println("= 5 || = 6 || = 7");
    }
    else {
      Serial.println("entered second loop");
      Serial.print("sensor A1: "); Serial.println(sensorA1);
      Serial.print("sensor A2: "); Serial.println(sensorA2);
      if (sensorA1 < 7) { // if sensorA1 is closer to wall than sensorA2
        moveRight(l_speed, r_speed, md, mv, 90);
        delay(50);
        moveForwardHug(lc_speed, rc_speed, md, mv, 5 - sensorA1);
        delay(50);
        moveLeft(l_speed, r_speed, md, mv, 90);
        delay(50);
        sensorA1 = (int) sensor.LFDistance(1); // left front
        sensorA2 = (int) sensor.LBDistance(1);
        sensorA1 = round(sensorA1);
        sensorA2 = round(sensorA2);
      }
      else {
        moveLeft(l_speed, r_speed, md, mv, 90);
        delay(50);
        //en.moveForwardHug(l_speed,r_speed,md,mv,sensorA1-5);
        md.setSpeeds(lc_speed, rc_speed);
        while ( (sensor.FLDistance(1) + sensor.FRDistance(1) ) / 2 > 6) {
          Serial.println(sensor.FLDistance(1));
          Serial.println(sensor.FRDistance(1));
          moveForwardHug(lc_speed, rc_speed, md, mv, 0.01);
        }
        md.setBrakes(300, 300);
        delay(50);
        moveRight(l_speed, r_speed, md, mv, 90);
        delay(50);
        sensorA1 = (int) sensor.LFDistance(1); // left front
        sensorA2 = (int) sensor.LBDistance(1);
        sensorA1 = round(sensorA1);
        sensorA2 = round(sensorA2);
      }
      checked = 0;
    }
    Serial.print("Sensor A1: "); Serial.println(sensorA1);
    Serial.print("Sensor A2: "); Serial.println(sensorA2);
  }  //md.setM1Brake(-250);
  //exit(1);
}

void Encoder::checkList1(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, int gridNum, Sensor sensor) {
  float distanceTraversed = 0;
  float distanceL = 0;
  float distanceR = 0;
  long tmpl_speed = 0;
  long tmpr_speed = 0;
  //resetDistance();
  rampUp(100, md, mv); //remember to change this when change rpm
  //md.setSpeeds(setLSpeed, setRSpeed);
  delay(20);
  //delay(2000);
  for ( int i = 0; i < gridNum; i++) {
    resetTicks();
    while (distanceTraversed <= 8.4) { // while havent reach distance, calibrate speed every 0.01seconds
      //delay(0.005); // should delay so the speed don't keep changing. need to tweak to get best interval
      tmpl_speed = mv.computeL(setLSpeed, insertionSort(10, ltime));
      tmpr_speed = mv.computeR(setRSpeed, insertionSort(10, rtime));
      md.setSpeeds(tmpl_speed, tmpr_speed);
      distanceL = 2 * 3 * M_PI * (getLticks() / 562.25);
      distanceR = 2 * 3 * M_PI * (getRticks() / 562.25);
      Serial.println(sensor.FLDistance(1));
      Serial.println(sensor.FMDistance(1));
      Serial.println(sensor.FRDistance(1));
      if ( sensor.FLDistance(1) <= 10 || sensor.FMDistance(1) <= 10 || sensor.FRDistance(1) <= 10) {
        md.setBrakes(300,300);
        moveRight(setLSpeed, setRSpeed, md, mv, 92);
        delay(1000);
        moveForward(setLSpeed, setRSpeed, md, mv, 2, sensor);
        delay(1000);
        moveLeft(setLSpeed, setRSpeed, md, mv, 90);
        delay(1000);
        moveForward(setLSpeed, setRSpeed, md, mv, 4, sensor);
        delay(1000);
        moveLeft(setLSpeed, setRSpeed, md, mv, 90);
        delay(1000);
        moveForward(setLSpeed, setRSpeed, md, mv, 2, sensor);
        delay(1000);
        moveRight(setLSpeed, setRSpeed, md, mv, 92);
        delay(1000);
      }
      distanceTraversed = (distanceL + distanceR) / 2;
    }
    distanceTraversed = 0;

  }
  md.setBrakes(300, 300);
  //resetDistance();

}

void Encoder::checkList2(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, int gridNum, Sensor sensor) {
  float distanceTraversed = 0;
  float distanceL = 0;
  float distanceR = 0;
  long tmpl_speed = 0;
  long tmpr_speed = 0;
  //resetDistance();
  rampUp(50, md, mv); //remember to change this when change rpm
  //md.setSpeeds(setLSpeed, setRSpeed);
  delay(20);
  //delay(2000);
  for ( int i = 0; i < gridNum; i++) {
    resetTicks();
    // because the distance tend to overshoot, it cant be a perfect 10, 9.6 works decent for 3 grids
    // for one grid the braking mechanism cannot stop in time. need take into account decceleration. 7.8 works good for 1 grid
    while (distanceTraversed <= 8.4) { // while havent reach distance, calibrate speed every 0.01seconds
      delay(0.005); // should delay so the speed don't keep changing. need to tweak to get best interval
      tmpl_speed = mv.computeL(setLSpeed, insertionSort(10, ltime));
      tmpr_speed = mv.computeR(setRSpeed, insertionSort(10, rtime));
      //Serial.print("tmpl_speed "); Serial.println(tmpl_speed);
      //Serial.print("tmpr_speed "); Serial.println(tmpr_speed);
      md.setSpeeds(tmpl_speed, tmpr_speed);
      distanceL = 2 * 3 * M_PI * (getLticks() / 562.25);
      distanceR = 2 * 3 * M_PI * (getRticks() / 562.25);
      Serial.println(sensor.FLDistance(1));
      Serial.println(sensor.FMDistance(1));
      Serial.println(sensor.FRDistance(1));
      if ( sensor.FLDistance(1) <= 20 || sensor.FMDistance(1) <= 20 || sensor.FRDistance(1) <= 20) {
        moveRight(setLSpeed, setRSpeed, md, mv, 45);
        delay(1000);
        moveForward(setLSpeed, setRSpeed, md, mv, 3, sensor);
        delay(1000);
        moveLeft(setLSpeed, setRSpeed, md, mv, 46);
        delay(1000);
        moveForward(setLSpeed, setRSpeed, md, mv, 2, sensor);
        delay(1000);
        moveLeft(setLSpeed, setRSpeed, md, mv, 46);
        delay(1000);
        moveForward(setLSpeed, setRSpeed, md, mv, 3, sensor);
        delay(1000);
        moveRight(setLSpeed, setRSpeed, md, mv, 45);
        delay(1000);
      }
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

    // Every 10cm output sensor values
  }
  md.setBrakes(300, 300);
  //resetDistance();

}

void Encoder::moveLoop(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, Sensor sensor) {
  long tmpl_speed = 0;
  long tmpr_speed = 0;
  int tmp_count = 0;
  float distanceTraversed = 0;
  float distanceL = 0;
  float distanceR = 0;
  rampUp(50, md, mv); //remember to change this when change rpm
  while(tmp_count < 4){
    while (sensor.FLDistance(1)>= 20 || sensor.FMDistance(1) >=20 || sensor.FRDistance(1) >=20 ) { // while havent reach distance, calibrate speed every 0.01seconds
      if( sensor.LBDistance(1) < 2.5 || sensor.LFDistance(1) < 2.5){
        wallHugging(setLSpeed, setRSpeed, md ,mv ,sensor);
      }
      if( distanceTraversed >= 10){
        distanceTraversed = 0;
        int flgrid = sensor.convertShort(sensor.FLDistance(1));
        int fmgrid = sensor.convertShort(sensor.FMDistance(1));
        int frgrid = sensor.convertShort(sensor.FRDistance(1));
        int lfgrid = sensor.convertShort(sensor.LFDistance(1));
        int lbgrid = sensor.convertShort(sensor.LBDistance(1));
    
        Serial.print(sensor.LBDistance(1)); Serial.print(" ");
        Serial.print(sensor.LFDistance(1)); Serial.print(" ");
        Serial.print(sensor.FLDistance(1)); Serial.print(" ");
        Serial.print(sensor.FMDistance(1)); Serial.print(" ");
        Serial.print(sensor.FRDistance(1)); Serial.print(" ");
        float rdistance = sensor.RDistance(1);
        int rgrid = sensor.convertLong(rdistance);      
        Serial.println(rdistance); Serial.print(" ");
     
        Serial.print(lbgrid); Serial.print(" ");
        Serial.print(lfgrid); Serial.print(" ");
        Serial.print(flgrid); Serial.print(" ");
        Serial.print(fmgrid); Serial.print(" ");
        Serial.print(frgrid); Serial.print(" ");     
        Serial.println(rgrid); Serial.print(" ");
      }
      distanceL = 6 * M_PI * (getLticks() / 562.25);
      distanceR = 6 * M_PI * (getRticks() / 562.25);
      distanceTraversed = (distanceL + distanceR) / 2;
      delay(100); // should delay so the speed don't keep changing. need to tweak to get best interval
      tmpl_speed = mv.computeL(setLSpeed, insertionSort(10, ltime));
      tmpr_speed = mv.computeR(setRSpeed, insertionSort(10, rtime));
      md.setSpeeds(tmpl_speed, tmpr_speed);
    }
    moveRight(setLSpeed, setRSpeed, md, mv, 90);    
    wallHugging(setLSpeed, setRSpeed, md ,mv ,sensor);    
    tmp_count++;

  }
  rampDown(100, md, mv); //remember to change this when change rpm
  //md.setBrakes(300, 300);
  //resetDistance();
}
