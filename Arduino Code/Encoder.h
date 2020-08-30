  static volatile long ltick; // volatile will tell the compiler that the value must be checked every time
  static volatile long rtick; 
  static unsigned long init_ltime; 
  static unsigned long init_rtime;  
  static unsigned long ltime;
  static unsigned long rtime;
class Encoder {

	private:
		int rpmTable[2][16];
	public: 
		//Left
		int pinA1;
		int pinA2;
		//Right
		int pinB1;
		int pinB2;		 
		Encoder(int, int, int, int);
		void init();
		void calibrate(int numIteration, DualVNH5019MotorShield md);
		void ltickIncrement();
		void rtickIncrement();
		void resetTicks();
		void tickCal(int numIteration, DualVNH5019MotorShield md);
		int getLticks();
		int getRticks();
		void calcRPM(unsigned long start_time);
		void stepLTest(DualVNH5019MotorShield md);
		void stepRTest(DualVNH5019MotorShield md);
		void stepPLTest(DualVNH5019MotorShield md);
		void stepPRTest(DualVNH5019MotorShield md);
    void moveForward(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv);
};	
