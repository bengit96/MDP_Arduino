#include "SharpIR.h"
#include "math.h"
#include "SensorCal.h"
#include "Arduino.h"

SensorCal::SensorCal(int irPin, long sensorModel) {
	
	_irPin=irPin;
    _model=sensorModel;
}

/* Total of 3 Methods available
    1. Using of the Library
    2. Using of the recommended equation
    3. A basic equation I've found that kinda works

    **Warning**
    Methods 2 & 3 does still need some tweaking
*/

float SensorCal::distance(int method) {
	SharpIR sensor(_irPin, _model);
	float distance;
	
	// Long Right
	if (_irPin == A0) {
		if (method == 1) {
			return distance = sensor.distance() - 5.84;
		}
		else if (method == 2) {
			float x = sensor.median_Voltage_Sampling();
			return distance = (278976.2-69729.29*x)/(1+3824.989*x-614.9018*(pow(x,2)));
		}
		else if (method == 3) {
			float x = sensor.median_Voltage_Sampling();
			return distance = (87.40818/x)-17.38074;
		}
		return -1;
	}
	
	// Short Left Back
	else if (_irPin == A1) {
		if (method == 1) {
			return distance = sensor.distance() - 5.34;
		}
		else if (method == 2) {
			float x = sensor.median_Voltage_Sampling();
			return distance = (64608.62-10629.52*x)/(1+1970.039*x+384.1247*(pow(x,2)));
		}
		else if (method == 3) {
			float x = sensor.median_Voltage_Sampling();
			return distance = (30.68132/x)-7.074208;
		}
		return -1;
	}
	
	// Short Left Front
	else if (_irPin == A2) {
		if (method == 1) {
			return distance = sensor.distance() - 5.34;
		}
		else if (method == 2) {
			float x = sensor.median_Voltage_Sampling();
			return distance = (159401.4-22277.58*x)/(1+3966.488*x+1617.304*(pow(x,2)));
		}
		else if (method == 3) {
			float x = sensor.median_Voltage_Sampling();
			return distance = (32.2953/x)-7.67349;
		}
		return -1;
	}
	
	// Short Front Middle
	else if (_irPin == A3) {
		if (method == 1) {
			return distance = sensor.distance() - 2.26; 
		}
		else if (method == 2) {
			float x = sensor.median_Voltage_Sampling();
			return distance = (342249.6-32728.63*x)/(1+12763.97*x+328.3559*(pow(x,2)));
		}
		else if (method == 3) {
			float x = sensor.median_Voltage_Sampling();
			return distance = (26.9723/x)-3.142919;
		}
		return -1;
	}
	
	
	// Short Front Left
	else if (_irPin == A4) {
		if (method == 1) {
			return distance = sensor.distance() - 3.91;
		}
		else if (method == 2) {
			float x = sensor.median_Voltage_Sampling();
			return distance = (163.3724-5.125353*x)/(1+2.455257*x+2.906178*(pow(x,2)));
		}
		else if (method == 3) {
			float x = sensor.median_Voltage_Sampling();
			return distance = (30.23257/x)-5.944718;
		}
		return -1;
	}
	
	// Short Front Right
	else if (_irPin == A5) {
		if (method == 1) {
			return distance = sensor.distance() - 3.92;
		}
		else if (method == 2) {
			float x = sensor.median_Voltage_Sampling();
			return distance = (228550.6-46173.84*x)/(1+7737.163*x-259.7944*(pow(x,2)));
		}
		else if (method == 3) {
			float x = sensor.median_Voltage_Sampling();
			return distance = (31.05792/x)-6.114549;
		}
		return -1;
	}
	
	return -1;
}