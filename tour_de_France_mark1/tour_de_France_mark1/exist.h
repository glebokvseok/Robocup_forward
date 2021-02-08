#ifndef EXIST
#define EXIST

#include <Arduino.h>
#include <avr/eeprom.h>
#include <Pixy2.h>
#include <Servo.h>

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
    		const double central_x = 162;
    		const double central_y = 109;
        const double coord_x = 157; // (164 62) (97 125) (216 109)
        const double coord_y = 122;
  	public:
    		int sign(double x);
    		double radian(double angle); 
    		double distance(int x, int y);
    		double countDistance(double d);
    		double countAngle(double ball_x, double ball_y);
};

class Robot {
  	private:
        const byte kicker_port = 11;
        const byte dribler_port = 8;
        const byte interruptor_port = 26;
    		const byte left_button_port = 23;
    		const byte right_button_port = 22;
    		const byte motors_in1[4] = {38, 42, 28, 3};
    		const byte motors_in2[4] = {36, 40, 30, 5};
    		const byte motors_pwm[4] = {6, 4, 10, 12};
        const byte led_digital_port[3] = {33, 35, 37};
        int calibration_value[24];
    		const double angle_coef = 0.785398163397448;
  	public:
    		int degree;
        Pixy2 pixy;
        Servo dribler;
        double led_angle[24];
        bool led_value[24];
  	public:
    		void init();
        void hitBall();
        bool checkHole();
        void updateGyro();
        void sendMail(String str);
        bool buttonPressed(byte n);
        void runDribler(int speed);
    		int readChannel(int n, int m);
    		bool updateLed(bool led_value[]);
    		void runMotor(byte port, short speed);
        bool setTimer(long long timer, int dt);
        int updateCamera(int signature[], int n);
    		void moveAngle(double angle, short speed, int u);
        String createMail(String name, double value, String str);
};

#endif
