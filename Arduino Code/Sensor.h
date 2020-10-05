class Sensor{
	private:
	public:
		Sensor();
		void rotateRight(int sensDiff, long l_speed, long r_speed);
		void rotateLeft(int sensDiff, long l_speed, long r_speed);
		void moveRight(int x,long l_speed,long r_speed);
		void moveLeft(int x,long l_speed,long r_speed);
		float RDistance(int method);
		float LBDistance(int method);
		float LFDistance(int method);
    float FLDistance(int method);
		float FMDistance(int method);
		float FRDistance(int method);
    int convertLong(float distance);
    int convertShort(float distance);
    void caliSensor();
};
