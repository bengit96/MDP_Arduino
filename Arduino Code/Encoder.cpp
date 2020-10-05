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

long unsigned int Encoder::bubbleSort(int numIteration, unsigned long * timeWidth) {
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
    rpmTable[0][i / -50 + 7] = bubbleSort(numIteration, timeWidth);
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
    rpmTable[1][i / 50 - 1] = bubbleSort(numIteration, timeWidth);
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

void Encoder::moveForwardHug(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, int distance) {
  float distanceTraversed = 0;
  float distanceL = 0;
  float distanceR = 0;
  long tmpl_speed = 0;
  long tmpr_speed = 0;
  //resetDistance();
  //rampUp(setLSpeed,setRSpeed,md);
  //Serial.print("SPEEDS");Serial.println(setLSpeed,setRSpeed);
  md.setSpeeds(setLSpeed, setRSpeed);
  //delay(20);
  //delay(2000);
  resetTicks();
  while (distanceTraversed <= distance) { // while havent reach distance, calibrate speed every 0.01seconds
    delay(0.005); // should delay so the speed don't keep changing. need to tweak to get best interval
    while(ltick < 10 || rtick < 10){
      tmpl_speed = mv.computeL(setLSpeed, bubbleSort(10, ltime));
      tmpr_speed = mv.computeR(setRSpeed, bubbleSort(10, rtime));
    }
    md.setSpeeds(tmpl_speed, tmpr_speed);
    distanceL = 2 * 3 * M_PI * (getLticks() / 562.25);
    distanceR = 2 * 3 * M_PI * (getRticks() / 562.25);

    distanceTraversed = (distanceL + distanceR) / 2;
  }
  md.setBrakes(300, 300);
  //resetDistance();

}

void Encoder::moveForward(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, int gridNum, Sensor sensor) {
  float distanceTraversed = 0;
  float distanceL = 0;
  float distanceR = 0;
  long tmpl_speed = 0;
  long tmpr_speed = 0;
  float setDistance = 8;
  //resetDistance();
  //md.setSpeeds(setLSpeed, setRSpeed);
  //delay(2000);
  for ( int i = 0; i < gridNum; i++) {
    resetTicks();
    /*
    if( i == 0){ // ramp up during first iteration
      rampUp(50, md, mv); 
    }
    */
    if ( i + 1 == gridNum) { //for last grid, setdistance smaller to accomodate braking
      setDistance = 8.3;
    }
    if(gridNum == 1){
      setDistance = 8.3; // batt 20
      //setDistance = 8.3; // batt 21
    }
    md.setSpeeds(setLSpeed, setRSpeed);
    while (distanceTraversed <= setDistance) { // while havent reach distance, calibrate speed every 0.01seconds
      
      if(sensor.FLDistance(1) < 2 || sensor.FRDistance(1) < 2 || sensor.FMDistance(1) <2){
        md.setBrakes(300, 300);
        i = gridNum;
        break;
      }
      
      delay(0.005); 
      while(ltick < 10 || rtick < 10){
        tmpl_speed = mv.computeL(setLSpeed, bubbleSort(10, ltime));
        tmpr_speed = mv.computeR(setRSpeed, bubbleSort(10, rtime));
      }
      md.setSpeeds(tmpl_speed, tmpr_speed);
      distanceL = 2 * 3 * M_PI * (getLticks() / 562.25);
      distanceR = 2 * 3 * M_PI * (getRticks() / 562.25);
      distanceTraversed = (distanceL + distanceR) / 2;
    }
    //Serial.print("distanceTraversed"); Serial.println(distanceTraversed);
    distanceTraversed = 0;
    if(gridNum == 1){
      md.setBrakes(300,300);
      delay(250);
    }else{
      //delay(50);
      if(sensor.LBDistance(3) < 10 && sensor.LFDistance(3) < 10){
        md.setBrakes(300,300);
        wallHugging(setLSpeed, setRSpeed, md ,mv ,sensor);
        delay(500);  
      }
    }
      Serial.print(sensor.LBDistance(3)); Serial.print(" ");
      Serial.print(sensor.LFDistance(3)); Serial.print(" ");
      Serial.print(sensor.FLDistance(1)); Serial.print(" ");
      Serial.print(sensor.FMDistance(1)); Serial.print(" ");
      Serial.print(sensor.FRDistance(1)); Serial.print(" ");      
      Serial.println(sensor.RDistance(1));
  }
  //rampDown(100, md, mv); //remember to change this when change rpm
  md.setBrakes(300, 300);
  //delay(1000);
  //resetDistance();

}


void Encoder::moveLeft(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, float degree, Sensor sensor, int cal) {
  float distanceTraversed = 0;
  float distanceL = 0;
  float distanceR = 0;
  long tmpl_speed = 0;
  long tmpr_speed = 0;
  int initial_delay = 20;
  float rpm_cali = 100;
  long lc_speed = mv.convertLSpeed(rpm_cali);
  long rc_speed = mv.convertRSpeed(rpm_cali);
  delay(150);
  resetTicks();
  //rampUp(100,100,md);
  //md.setSpeeds(-setLSpeed, setRSpeed);
  //delay(initial_delay);
  //0.0000004768372
  //while ( distanceTraversed <= (15.27887395921 * M_PI /4 - 0.000000476837186624835)/90 * degree ) { // while havent reach distance, calibrate speed every 0.01seconds
  while ( distanceTraversed <= (15 * M_PI / 4) / 90 * degree ) {
    delay(0.005); // should delay so the speed don't keep changing. need to tweak to get best interval
    tmpl_speed = mv.computeL(lc_speed, bubbleSort(10, ltime));
    tmpr_speed = mv.computeR(rc_speed, bubbleSort(10, rtime));
    md.setSpeeds(-tmpl_speed, tmpr_speed);
    distanceL = 6 * M_PI * (getLticks() / 562.25);
    distanceR = 6 * M_PI * (getRticks() / 562.25);
    distanceTraversed = (distanceL + distanceR) / 2;
  }

  md.setBrakes(300, 300);
    //delay(1000);    //Serial.print("PC,AR,");
    if(cal != 1){
      delay(500);
      Serial.print(sensor.LBDistance(3)); Serial.print(" ");
      Serial.print(sensor.LFDistance(3)); Serial.print(" ");
      Serial.print(sensor.FLDistance(1)); Serial.print(" ");
      Serial.print(sensor.FMDistance(1)); Serial.print(" ");
      Serial.print(sensor.FRDistance(1)); Serial.print(" ");      
      Serial.println(sensor.RDistance(1));
    }
    
}

void Encoder::moveRight(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, float degree, Sensor sensor, int cal) {
  float distanceTraversed = 0;
  float distanceL = 0;
  float distanceR = 0;
  int initial_delay = 20;
  float rpm_cali = 100;
  long lc_speed = mv.convertLSpeed(rpm_cali);
  long rc_speed = mv.convertRSpeed(rpm_cali);
  resetTicks();
  if(cal != 1){
    
    if(sensor.FLDistance(1) < 10){
      while(sensor.FLDistance(1) > 7){
        moveForwardHug(setLSpeed, setRSpeed, md, mv, sensor.FLDistance(1)-7);
      }
    }
    
    else if(sensor.FMDistance(1) < 10){
      while(sensor.FMDistance(1) > 7){
        moveForwardHug(setLSpeed, setRSpeed, md, mv, sensor.FMDistance(1)-7);
      }
    }
    else if(sensor.FRDistance(1) < 10){
      while(sensor.FRDistance(1) > 7){
        moveForwardHug(setLSpeed, setRSpeed, md, mv, sensor.FRDistance(1)-7);   
      }
    }
    
    if(sensor.LFDistance(1) <= 10 && sensor.LBDistance(1) <= 10 ){
      wallHugging(setLSpeed, setRSpeed, md ,mv ,sensor);  
      delay(150);
    } 
  //delay(500);
    
  }

  resetTicks();
  //   while ( distanceTraversed <= (15.27887395921 * M_PI /4 - 0.000000476837186624835)/90 * degree) { // while havent reach distance, calibrate speed every 0.01seconds
  while ( distanceTraversed <= (15 * M_PI / 4 ) / 90 * degree ) {
    delay(0.005); // should delay so the speed don't keep changing. need to tweak to get best interval
    long tmpl_speed = mv.computeL(lc_speed, bubbleSort(10, ltime));
    long tmpr_speed = mv.computeR(rc_speed, bubbleSort(10, rtime));
    md.setSpeeds(tmpl_speed, -tmpr_speed);
    distanceL = 2 * 3 * M_PI * (getLticks() / 562.25);
    distanceR = 2 * 3 * M_PI * (getRticks() / 562.25);
    distanceTraversed = (distanceL + distanceR) / 2;
  }
  md.setBrakes(300, 300);
   // delay(1000);    //Serial.print("PC,AR,");
    if(cal != 1){
      delay(500);
      Serial.print(sensor.LBDistance(3)); Serial.print(" ");
      Serial.print(sensor.LFDistance(3)); Serial.print(" ");
      Serial.print(sensor.FLDistance(1)); Serial.print(" ");
      Serial.print(sensor.FMDistance(1)); Serial.print(" ");
      Serial.print(sensor.FRDistance(1)); Serial.print(" ");      
      Serial.println(sensor.RDistance(1));
    }
}

void Encoder::moveLeftHug(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv) {
  float distanceTraversed = 0;
  float distanceL = 0;
  float distanceR = 0;
  long tmpl_speed = 0;
  long tmpr_speed = 0;
  int initial_delay = 20;
  resetTicks();
  //0.0000004768372
  while ( distanceTraversed <= 0.01 ) { // while havent reach distance, calibrate speed every 0.01seconds
    md.setSpeeds(-setLSpeed, setRSpeed);
    distanceL = 2 * 3 * M_PI * (getLticks() / 562.25);
    distanceR = 2 * 3 * M_PI * (getRticks() / 562.25);
    distanceTraversed = (distanceL + distanceR) / 2;
  }
  md.setBrakes(300, 300);
  //moveForward(setLSpeed, setRSpeed, md, mv, 1);
}

void Encoder::moveRightHug(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv) {
  float distanceTraversed = 0;
  float distanceL = 0;
  float distanceR = 0;
  long tmpl_speed = 0;
  long tmpr_speed = 0;
  resetTicks();
  //0.0000004768372
  while ( distanceTraversed <= 0.01 ) { // while havent reach distance, calibrate speed every 0.01seconds
    md.setSpeeds(setLSpeed, -setRSpeed);
    distanceL = 2 * 3 * M_PI * (getLticks() / 562.25);
    distanceR = 2 * 3 * M_PI * (getRticks() / 562.25);
    distanceTraversed = (distanceL + distanceR) / 2;
  }
  md.setBrakes(300, 300);
}

void  Encoder:: wallHugging(long l_speed, long r_speed, DualVNH5019MotorShield md, Movement mv, Sensor sensor) {
  int checked = 0;
  int cond = 0;
  float rpm_cali = 8;
  long lc_speed = mv.convertLSpeed(rpm_cali);
  long rc_speed = mv.convertRSpeed(rpm_cali);
  int shake = 0;
  //int shake = 0;
  //Check if front and back is aligned
  while (checked == 0) {
    float sensorA1 = sensor.LFDistance(3);
    float sensorA2 = sensor.LBDistance(3); 

    checked = 1; // if entered either of the loops, check again
    //shake = 0;
    while ( abs(sensorA1 - sensorA2) >= 0.03) {
      rpm_cali = 8;
      /*
      shake++;
      if(shake >= 5){
        rpm_cali = 6;
      }else{
        rpm_cali = 20;
      }
      */
      /*
      shake++;
      if(shake >= 50){
        checked = 0;
        break;
      }
      */
      //rpm_cali = 15;
      //rpm_cali = 20;
      lc_speed = mv.convertLSpeed(rpm_cali);
      rc_speed = mv.convertRSpeed(rpm_cali);      
      if (sensorA1 < sensorA2) { // if sensorA1 is closer to wall than sensorA2
        moveRightHug(lc_speed, rc_speed, md, mv);
        sensorA1 = sensor.LFDistance(3); // left front
        sensorA2 = sensor.LBDistance(3);
        //sensorA1 = round(sensorA1);
        //sensorA2 = round(sensorA2);
      }
      else {
        moveLeftHug(lc_speed, rc_speed, md, mv);
        sensorA1 = sensor.LFDistance(3); // left front
        sensorA2 = sensor.LBDistance(3);
        //sensorA1 = round(sensorA1);
        //sensorA2 = round(sensorA2);
      }
      checked = 0;
    }
    //sensorA1 = round(sensorA1);
    //sensorA2 = round(sensorA2);
        //Serial.print("sensor left front: "); Serial.println(sensorA1);
        //Serial.print("sensor left back: "); Serial.println(sensorA2);

    if ((int) sensorA1 == 5 || (int)sensorA1 == 6 || (int)  sensorA1 == 7) {
      //Serial.println("= 5 || = 6 || = 7");
    }
    else {
      //shake = 0;
    rpm_cali = 20;
      //rpm_cali = 40;
    lc_speed = mv.convertLSpeed(rpm_cali);
    rc_speed = mv.convertRSpeed(rpm_cali);
      //Serial.println("entered second loop");
      //Serial.print("sensor A1: "); Serial.println(sensorA1);
      //Serial.print("sensor A2: "); Serial.println(sensorA2);
      if (sensorA1 < 7) { // if sensorA1 is closer to wall than sensorA2
        moveRight(l_speed, r_speed, md, mv, 83,sensor,1);
        delay(50);
        moveForwardHug(lc_speed, rc_speed, md, mv, 5 - sensorA1);
        delay(50);
        moveLeft(l_speed, r_speed, md, mv, 83,sensor,1);
        delay(50);
        sensorA1 = (int) sensor.LFDistance(3); // left front
        sensorA2 = (int) sensor.LBDistance(3);
        sensorA1 = round(sensorA1);
        sensorA2 = round(sensorA2);
      }
      else {
        moveLeft(l_speed, r_speed, md, mv, 83,sensor,1);
        delay(50);
        //en.moveForwardHug(l_speed,r_speed,md,mv,sensorA1-5);
        md.setSpeeds(lc_speed, rc_speed);
        cond = 0;
        while ( sensor.FLDistance(1)> 6 || sensor.FRDistance(1) > 6) {
          moveForwardHug(lc_speed, rc_speed, md, mv, 0.01);
          if(sensor.FLDistance(1) < 6 || sensor.FRDistance(1) < 6){
            break;
          }
        }
        md.setBrakes(300, 300);
        delay(50);
        moveRight(l_speed, r_speed, md, mv, 83,sensor,1);
        delay(50);
        sensorA1 = (int) sensor.LFDistance(3); // left front
        sensorA2 = (int) sensor.LBDistance(3);
        sensorA1 = round(sensorA1);
        sensorA2 = round(sensorA2);
      }
      //delay(150);
      checked = 0;
    }
  } 
  //delay(150);
  //Serial.println(sensor.LFDistance(3));
  //Serial.println(sensor.LBDistance(3));
}

int Encoder::moveForwardGoal(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, int gridNum, Sensor sensor, int hori) {
  float distanceTraversed = 0;
  float distanceL = 0;
  float distanceR = 0;
  long tmpl_speed = 0;
  long tmpr_speed = 0;
  float setDistance = 8;
  setLSpeed = mv.convertLSpeed(100); // change this functions based on gradient found and y intercept
  setRSpeed = mv.convertRSpeed(100); // change this functions based on gradient found and y intercept
  //resetDistance();
  //md.setSpeeds(setLSpeed, setRSpeed);
  //delay(2000);
  for ( int i = 0; i < gridNum; i++) {
    resetTicks();
    if(hori == 0){
      if ( i + 1 == gridNum) { 
        setDistance = 10;
      }
      if(gridNum == 1){
        setDistance = 10;
      }
    }
    else{
      if(gridNum == 1){
        setDistance = 14.4;
      } else{
        setDistance = 13.5;
      }
    }

    md.setSpeeds(setLSpeed, setRSpeed);
    while (distanceTraversed <= setDistance) { // while havent reach distance, calibrate speed every 0.01seconds
      /*
      if(sensor.FLDistance(1) < 6 || sensor.FRDistance(1) < 6 || sensor.FMDistance(1) <6){
        md.setBrakes(300, 300);
        return 0;
      }
      */
      delay(0.005); 
      while( (ltick < 10 || rtick < 10)){
        tmpl_speed = mv.computeL(setLSpeed, bubbleSort(10, ltime));
        tmpr_speed = mv.computeR(setRSpeed, bubbleSort(10, rtime));
      }
      //tmp++;
      md.setSpeeds(tmpl_speed, tmpr_speed);
      distanceL = 2 * 3 * M_PI * (getLticks() / 562.25);
      distanceR = 2 * 3 * M_PI * (getRticks() / 562.25);
      distanceTraversed = (distanceL + distanceR) / 2;
      
    }
    distanceTraversed = 0;
    //Serial.println("d");
  }
  md.setBrakes(300, 300);
}



/*
int Encoder::moveForwardGoal(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, int gridNum, Sensor sensor){
  float distanceTraversed = 0;
  float distanceL = 0;
  float distanceR = 0;
  long tmpl_speed = 0;
  long tmpr_speed = 0;
  float setDistance = 0;
  int tmp = 0;
  //resetDistance();
  //md.setSpeeds(setLSpeed, setRSpeed);
  //delay(2000);
    resetTicks();
    if ( i + 1 == gridNum) { //for last grid, setdistance smaller to accomodate braking
      setDistance = 8.5;
    }

    if(gridNum == 10){
      setDistance = 8.4;
    }
    else{
      setDistance = gridNum - (1.4 * ((int) gridNum/10));
    }
    md.setSpeeds(setLSpeed, setRSpeed);
    while (distanceTraversed <= setDistance) { // while havent reach distance, calibrate speed every 0.01seconds
      //if(tmp = 10){
        if(sensor.LFDistance(1) <= 10 && sensor.LBDistance(1) <= 10 ){
          wallHugging(setLSpeed, setRSpeed, md ,mv ,sensor);  
          delay(100);
        }    
      //}   
      if(sensor.FLDistance(1) < 2 || sensor.FRDistance(1) < 2 || sensor.FMDistance(1) <2){
        md.setBrakes(300, 300);
        return -1;
        break;
      }   
      delay(0.005); 
      while(ltick < 10 || rtick < 10){
        tmpl_speed = mv.computeL(setLSpeed, bubbleSort(10, ltime));
        tmpr_speed = mv.computeR(setRSpeed, bubbleSort(10, rtime));
      }   
      md.setSpeeds(tmpl_speed, tmpr_speed);
      distanceL = 2 * 3 * M_PI * (getLticks() / 562.25);
      distanceR = 2 * 3 * M_PI * (getRticks() / 562.25);
      distanceTraversed = (distanceL + distanceR) / 2;
    }
    //Serial.print("distanceTraversed"); Serial.println(distanceTraversed);
    distanceTraversed = 0;
  
  //rampDown(100, md, mv); //remember to change this when change rpm
  md.setBrakes(300, 300);
  return 1;
  //resetDistance();

}

*/
/*
void Encoder::rampUp(long rpm, DualVNH5019MotorShield md, Movement mv) {
  long tmplspeed = mv.convertRSpeed(1);
  long tmprspeed = mv.convertLSpeed(1);
  long tmp_rpm = 0;
  while (tmp_rpm < rpm) {
    tmplspeed = mv.convertRSpeed(tmp_rpm);
    tmprspeed = mv.convertLSpeed(tmp_rpm);
    tmp_rpm++;
    delay(10);
    md.setSpeeds(-tmplspeed, -tmprspeed);
  }
}

void Encoder::rampDown(long rpm, DualVNH5019MotorShield md, Movement mv) {
  long tmplspeed = mv.convertRSpeed(1);
  long tmprspeed = mv.convertLSpeed(1);
  //long tmp_rpm = 0;
  while (rpm != 0) {
    tmplspeed = mv.convertRSpeed(rpm);
    tmprspeed = mv.convertLSpeed(rpm);
    rpm -= 2;
    md.setSpeeds(tmplspeed, tmprspeed);
  }
  md.setBrakes(300, 300);
}


*/