#include <SharpIR.h>
#include <math.h>

// Sensors
#define ps1 A0 //Large Right
#define ps2 A1 //Small Left Back
#define ps3 A2 //Small Left Front
#define ps4 A3 //Small Front Middle
#define ps5 A4 //Small Front Left
#define ps6 A5 //Small Front Right

SharpIR right_sensor(ps1, 20150);
SharpIR left_back_sensor(ps2, 1080);
SharpIR left_front_sensor(ps3, 1080);
SharpIR front_middle_sensor(ps4, 1080);
SharpIR front_left_sensor(ps5, 1080);
SharpIR front_right_sensor(ps6, 1080);

// Methods
float frontMiddle_Distance(int method) {
  float distance;
  if (method == 1) {
    return distance = front_middle_sensor.distance() - 2.26; 
  }
  else if (method == 2) {
    float x = front_middle_sensor.median_Voltage_Sampling();
    return distance = (342249.6-32728.63*x)/(1+12763.97*x+328.3559*(pow(x,2)));
  }
  else if (method == 3) {
    float x = front_middle_sensor.median_Voltage_Sampling();
    return distance = (26.9723/x)-3.142919;
  }
  return -1;
}

float frontLeft_Distance(int method) {
  float distance;
  if (method == 1) {
    return distance = front_left_sensor.distance() - 3.91;
  }
  else if (method == 2) {
    float x = front_left_sensor.median_Voltage_Sampling();
    return distance = (163.3724-5.125353*x)/(1+2.455257*x+2.906178*(pow(x,2)));
  }
  else if (method == 3) {
    float x = front_left_sensor.median_Voltage_Sampling();
    return distance = (30.23257/x)-5.944718;
  }
  return -1;
}

float frontRight_Distance(int method) {
  float distance;
  if (method == 1) {
    return distance = front_right_sensor.distance() - 3.92;
  }
  else if (method == 2) {
    float x = front_right_sensor.median_Voltage_Sampling();
    return distance = (228550.6-46173.84*x)/(1+7737.163*x-259.7944*(pow(x,2)));
  }
  else if (method == 3) {
    float x = front_right_sensor.median_Voltage_Sampling();
    return distance = (31.05792/x)-6.114549;
  }
  return -1;
}

float leftFront_Distance(int method) {
  float distance;
  if (method == 1) {
    return distance = left_front_sensor.distance() - 5.34;
  }
  else if (method == 2) {
    float x = left_front_sensor.median_Voltage_Sampling();
    return distance = (159401.4-22277.58*x)/(1+3966.488*x+1617.304*(pow(x,2)));
  }
  else if (method == 3) {
    float x = left_front_sensor.median_Voltage_Sampling();
    return distance = (32.2953/x)-7.67349;
  }
  return -1;
}

float leftBack_Distance(int method) {
  float distance;
  if (method == 1) {
    return distance = left_back_sensor.distance() - 5.34;
  }
  else if (method == 2) {
    float x = left_back_sensor.median_Voltage_Sampling();
    return distance = (64608.62-10629.52*x)/(1+1970.039*x+384.1247*(pow(x,2)));
  }
  else if (method == 3) {
    float x = left_back_sensor.median_Voltage_Sampling();
    return distance = (30.68132/x)-7.074208;
  }
  return -1;
}

float right_Distance(int method) {
  float distance;
  if (method == 1) {
    return distance = right_sensor.distance() - 5.84;
  }
  else if (method == 2) {
    float x = right_sensor.median_Voltage_Sampling();
    return distance = (278976.2-69729.29*x)/(1+3824.989*x-614.9018*(pow(x,2)));
  }
  else if (method == 3) {
    float x = right_sensor.median_Voltage_Sampling();
    return distance = (87.40818/x)-17.38074;
  }
  return -1;
}

void setup() {
  Serial.begin(9600);
}

/* Total of 3 Methods available
    1. Using of the Library
    2. Using of the recommended equation
    3. A basic equation I've found that kinda works

    **Warning**
    Methods 2 & 3 does still need some tweaking
  */

void loop() {
  float distance = frontMiddle_Distance(3);
  Serial.print("Distance: ");Serial.println(distance);
  delay(1000);
}
