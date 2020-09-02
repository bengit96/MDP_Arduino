#include <SensorCal.h>

// Sensors
#define ps1 A0 //Large Right
#define ps2 A1 //Small Left Back
#define ps3 A2 //Small Left Front
#define ps4 A3 //Small Front Middle
#define ps5 A4 //Small Front Left
#define ps6 A5 //Small Front Right

SensorCal right_sensor(ps1, 20150);
SensorCal left_back_sensor(ps2, 1080);
SensorCal left_front_sensor(ps3, 1080);
SensorCal front_middle_sensor(ps4, 1080);
SensorCal front_left_sensor(ps5, 1080);
SensorCal front_right_sensor(ps6, 1080);

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
  float distance = front_middle_sensor.distance(2);
  Serial.print("Distance: ");Serial.println(distance);
  delay(1000);
}
