/* 
	Sharp IR Sensor Calibration
	
	Arduino Library to calibrate the sensors for our MDP project
	
	Version : 1.0 : Aleem Siddique	
*/

#ifndef SensorCal_h
#define SensorCal_h

#include "Arduino.h"
#include "SharpIR.h"

class SensorCal 
{
	public:
	
		SensorCal (int irPin, long sensorModel);
		float distance(int method);
	
	private:
	
		int _irPin;
		long _model;
		
};

#endif