long static errorL[2] = {-1};
long static errorR[2] = {-1};


class Movement {

	private:
		float lKp, lKi, lKd;
    float rKp, rKi, rKd;
		float l_speed, r_speed;
	public: 
    float distanceL = 0;
    float distanceR = 0;
    float distanceTraversed = 0;
    float previousLSpeed = 0;
    float previousLError = 0;
    float previousLError2 = 0;
    float previousRSpeed = 0;
    float previousRError = 0;
    float previousRError2 = 0;
		//Left
		int pinA1;
		int pinA2;
		//Right
		int pinB1;
		int pinB2;		 
		Movement(int, int, int, int, float, float, float,float,float,float);
		float computeL(long setLSpeed, unsigned long ltime);
		float computeR(long setRSpeed, unsigned long rtime);
		//void moveForward(long setLSpeed, long setRSpeed, Encoder en , DualVNH5019MotorShield md);
    float convertLSpeed(float rpm);
    float convertRSpeed(float rpm);
    void resetDistance();
};	
