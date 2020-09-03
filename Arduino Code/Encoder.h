  static volatile long ltick; // volatile will tell the compiler that the value must be checked every time
  static volatile long rtick; 
  static unsigned long init_ltime; 
  static unsigned long init_rtime;  
  static unsigned long ltime[20];
  static unsigned long rtime[20];
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
    void stepLTest(DualVNH5019MotorShield md, int timeWidth);
    void stepRTest(DualVNH5019MotorShield md, int timeWidth);		
		int getLticks();
		int getRticks();
		void calcRPM(unsigned long start_time);
		void stepPLTest(DualVNH5019MotorShield md);
		void stepPRTest(DualVNH5019MotorShield md);
    void moveForward(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, int gridNum);
    void moveLeft(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, float degree);
    void moveRight(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, float degree);
    void printTime();
    void rampUp(long rpm, DualVNH5019MotorShield md, Movement m);
    void moveForwardHug(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, int distance);
    void moveBackHug(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, int distance);
    long unsigned int bubbleSort(int numIteration, unsigned long * timeWidth);
    void moveLeftHug(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv);
    void moveRightHug(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv);
};	
