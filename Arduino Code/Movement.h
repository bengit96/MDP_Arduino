

  static volatile long ltick; // volatile will tell the compiler that the value must be checked every time
  static volatile long rtick; 
  static unsigned long init_ltime; 
  static unsigned long init_rtime;  
  static unsigned long ltime[10];
  static unsigned long rtime[10];
class Movement {

	private:
	public: 
		//Left
		int pinA1;
		int pinB1;
		//Right
		int pinA2;
		int pinB2;		 
		Movement(int, int, int, int);
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
		void stepPLTest(DualVNH5019MotorShield md);
		void stepPRTest(DualVNH5019MotorShield md);
    void moveForward(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, PID pid, int gridNum, Sensor sensor);
    int moveForwardGoal(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, PID pid, int gridNum, Sensor sensor,int hori, int last);
    //int moveForwardGoal(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, int gridNum, Sensor sensor);
    //void moveLoop(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, Sensor sensor);
    void moveLeft(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, PID pid, float degree, Sensor sensor, int cal);
    void moveRight(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, PID pid, float degree, Sensor sensor, int cal);
    //void rampUp(long rpm, DualVNH5019MotorShield md, Movement m);
    //void rampDown(long rpm, DualVNH5019MotorShield md, Movement m);
    void moveForwardHug(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, PID pid, int distance);
    long unsigned int bubbleSort(int numIteration, unsigned long * timeWidth);
    void moveLeftHug(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, PID pid);
    void moveRightHug(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, PID pid);
    void wallHugging(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, PID pid, Sensor sensor);
    //void checkList1(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, int gridNum, Sensor sensor);
    //void checkList2(long setLSpeed, long setRSpeed, DualVNH5019MotorShield md, Movement mv, int gridNum, Sensor sensor);

};	
