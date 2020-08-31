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
            const double central_x = 169;
            const double central_y = 96;
            const double front_x = 158; 
            const double front_y = 100; 
    public:
            int sign(double x);
            double radian(double angle); 
            double distance(int x, int y);
            double countDistance(double d);
            double countAngle(double ball_x, double ball_y);
};

class Robot {
    private:
        const byte interruptor_port = 26;
            const byte left_button_port = 23;
            const byte right_button_port = 22;
            const byte motors_in1[4] = {38, 42, 28, 3};
            const byte motors_in2[4] = {36, 40, 30, 5};
            const byte motors_pwm[4] = {6, 44, 10, 12};
        const byte led_digital_port[3] = {33, 35, 37};
        int calibration_value[24] = {515, 520, 535, 520, 520, 486, 538, 600, 522, 539, 443, 536, 551, 508, 469, 531, 596, 563, 477, 572, 467, 477, 438, 507};
            const double angle_coef = 0.785398163397448;
    public:
            int degree;
        Pixy2 pixy;
        double led_angle[24];
        bool led_value[24];
    public:
            void init();
        bool checkHole();
        void updateGyro();
        void sendMail(String str);
        bool buttonPressed(byte n);
            int readChannel(int n, int m);
            bool updateLed(bool led_value[]);
            void runMotor(byte port, short speed);
        bool setTimer(long long timer, int dt);
        int updateCamera(int signature[], int n);
            void moveAngle(double angle, short speed, int u);
        String createMail(String name, double value, String str);
};

#endif