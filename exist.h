#ifndef EXIST
#define EXIST

#include <Arduino.h>
#include <Pixy2.h>

struct camera {
		int x;
		int y;
		int width;
		int height;
		double angle;
		double distance;
		double square;
		bool found;
};

class Math {
	private:
		const double central_x = 164;
		const double central_y = 115;
		const double front_x = 175; 
		const double front_y = 116; 
	public:
		int sign(double x);
		double radian(double angle); 
		double distance(int x, int y);
		double countDistance(double d);
		double countAngle(double ball_x, double ball_y);
};

class Robot {
	private:
		const byte left_button_port = 23;
		const byte right_button_port = 22;
		const byte led_digital_port[3] = {33, 35, 37};
		const byte motors_in1[4] = {38, 42, 28, 3};
		const byte motors_in2[4] = {36, 40, 30, 5};
		const byte motors_pwm[4] = {6, 44, 10, 12};
    int calibration_value[24] = {581, 605, 582, 605, 586, 547, 622, 687, 629, 632, 542, 644, 630, 596, 551, 591, 657, 639, 544, 643, 535, 539, 520, 571};
		double led_angle[24];
		const double angle_coef = 0.785398163397448;

	public:
		int degree;
    Pixy2 pixy;
	public:
		void init();
		bool buttonPressed(char);
		int readChannel(int n, int m);
		double updateLed();
		void runMotor(byte port, short speed);
		void moveAngle(double angle, short speed, int u);
		void updateGyro();
    int updateCamera(int signature[], int n);
    bool setTimer(long long timer, int dt);
};

#endif
