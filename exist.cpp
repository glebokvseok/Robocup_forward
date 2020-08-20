#include "exist.h"

#include <Arduino.h>
#include <Pixy2.h>

int Math::sign(double x) {
   if (x >= 0) {
       return 1;
   } else {
       return -1;
   }
}

double Math::radian(double angle) {
      return angle * PI / 180;
}

double Math::distance(int x, int y) {
    return sqrt(pow(x - central_x, 2) + pow(y - central_y, 2));
}

double Math::countDistance(double d) {
    return ((abs(1.11333 * pow(d, 3) - 12.745 * pow(d, 2) + 52.8417 * pow(d, 1) - 5.15)) / 1000);
}

double Math::countAngle(double ball_x, double ball_y) {
   double vec_ax = this->front_x - this->central_x; 
   double vec_ay = this->front_y - this->central_y; 
   double vec_bx = ball_x - central_x; 
   double vec_by = ball_y - central_y; 
   double angle = atan2(vec_by - vec_ay, vec_bx - vec_ay);
   return angle;
}

bool Robot::setTimer(long long timer, int dt) {
    return(millis() - timer > dt);
}

void Robot::init() {
      Serial.begin(115200);
    Serial1.begin(9600);
    Serial3.begin(115200);
    delay(4000);     
    Serial3.write(0XA5);
    Serial3.write(0X54);
    delay(4000);
    Serial3.write(0XA5);
    Serial3.write(0X51);

    pinMode(this->left_button_port, INPUT_PULLUP);
    pinMode(this->right_button_port, INPUT_PULLUP);
    for (int i = 0; i < 3; ++i) {
        pinMode(this->led_digital_port[i], OUTPUT);
        digitalWrite(this->led_digital_port[i], LOW);
    }
    for (int i = 0; i < 4; ++i) {
        pinMode(this->motors_pwm[i], OUTPUT);
        pinMode(this->motors_in1[i], OUTPUT);
        pinMode(this->motors_in2[i], OUTPUT);
    }
    for (int j = 0; j < 4; ++j) { 
        for (int i = 0; i < 6; ++i) { 
            this->led_angle[j * 6 + i] = (j * 6 + i) * 15 * PI / 180 - 30 * PI / 180;
        }
    }
    for (int i = 0; i < 23; ++i) {
        this->led_value[i] = false;
    }
    this->led_angle[0] += 2 * PI;
    this->led_angle[1] += 2 * PI;
}

String Robot::createMail(String name, double value, String str) {
    str = str + name + " = " +  String(value) + "|";
    return str;
}

void Robot::sendMail(String str) {
    str += "!";
    Serial1.print(str);
}

bool Robot::buttonPressed(byte n) {
    int button_port;
    if (n == 0)
        button_port = this->left_button_port;
    if (n == 1)
        button_port = this->right_button_port;
        return digitalRead(button_port) == 0;
}

int Robot::readChannel(int n, int m) {
    int control_pins[3] = {33, 35, 37};
    int signal[4] = {A3, A5, A7, A1}; 
    int channels[6][3] = { 
      {0, 0, 0},
      {1, 0, 0},
      {0, 1, 0},
      {1, 1, 0},
      {0, 0, 1},
      {1, 0, 1}
    };
    for (int i = 0; i < 3; ++i) {
        digitalWrite(control_pins[i], channels[n][i]);
    }
    int value = analogRead(signal[m]);
    return value;
}

bool Robot::updateLed(bool led_value[]) {
    for (int j = 0; j < 4; ++j) { 
        for (int i = 0; i < 6; ++i) { 
            if (readChannel(i, j) > this->calibration_value[j * 6 + i])
                led_value[j * 6 + i] = true;
        }
    }
    return led_value;
}

void Robot::runMotor(byte port, short speed) {
    if (speed > 255) 
        speed = 255;
    if (speed < -255)
        speed = -255;
        digitalWrite(this->motors_in1[port], speed > 0);
        digitalWrite(this->motors_in2[port], !(speed > 0));
        analogWrite(this->motors_pwm[port], abs(speed));
}

void Robot::moveAngle(double angle, short speed, int u) {
    runMotor(0, speed * sin(angle + this->angle_coef) + u); 
    runMotor(1, speed * sin(angle - this->angle_coef) + u);
    runMotor(2, speed * sin(angle + this->angle_coef) - u);
    runMotor(3, speed * sin(angle - this->angle_coef) - u);
}

int Robot::updateCamera(int signature[], int n) {
    int maxS[3] = {0, 0, 0};
    for (int i = 0; i < n; ++i) {
        if (maxS[this->pixy.ccc.blocks[i].m_signature - 1] < this->pixy.ccc.blocks[i].m_width * this->pixy.ccc.blocks[i].m_height) {
            maxS[this->pixy.ccc.blocks[i].m_signature - 1] = this->pixy.ccc.blocks[i].m_width * this->pixy.ccc.blocks[i].m_height;
            signature[this->pixy.ccc.blocks[i].m_signature - 1] = i;
        }
    }
    return signature;
}

void Robot::updateGyro() {
    unsigned char Re_buf[8];
    long long counter = 0;
    Serial3.write(0XA5);
    Serial3.write(0X51); //send it for each read
    while (Serial3.available()) {   
        Re_buf[counter] = (unsigned char)Serial3.read();
        if (counter == 0 && Re_buf[0] != 0xAA) return;       
            ++counter;       
        if(counter == 8) {   
            counter = 0;                 
            if (Re_buf[0] == 0xAA && Re_buf[7] == 0x55) {  // data package is correct        
                   this->degree = (int16_t)(Re_buf[1]<<8|Re_buf[2]) / 100.00;   
            }     
        } 
    }
}