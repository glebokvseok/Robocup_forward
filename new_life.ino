#include "3xist.h"

Robot robot;
Math math;

camera ball, home, enemy;
const double epsilon = pow(10, -6);

long long int timer = 0;
long long int line_timer = 0;
long long int delay_timer = 0;
long long int serial_timer = 0;
long long int switch_update_timer = 0;

bool ball_catched = false;
bool line_catched = false;
bool mirror_exist = false;
bool switched_goals = false;
bool mirror_line_catched = false;

double alpha = -1.0;
double alpha1 = -1.0;
double alpha2 = -1.0;
double line_angle = 0.0;
double current_angle = 0.0;

int target = 0;

void setup() {
    robot.init();
    robot.pixy.init();
    robot.updateGyro();
    target = robot.degree;
}

void loop() {
    short speed = 240;
    short led_counter = 0;
    
    robot.pixy.ccc.getBlocks();

    if (robot.pixy.ccc.numBlocks) {
        int signature[3] = {-1, -1, -1};
        
        robot.updateCamera(signature, robot.pixy.ccc.numBlocks);
        
        if (switched_goals) {
            int tmp = signature[1];
            signature[1] = signature[2];
            signature[2] = tmp;
        }
        
        ball.found = (signature[0] != -1);
        enemy.found = (signature[1] != -1);
        home.found = (signature[2] != -1);

        if (ball.found) {
            ball.x = robot.pixy.ccc.blocks[signature[0]].m_x; 
            ball.y = robot.pixy.ccc.blocks[signature[0]].m_y;
            ball.angle = math.countAngle(ball.x, ball.y);
            ball.square = robot.pixy.ccc.blocks[signature[0]].m_width * robot.pixy.ccc.blocks[signature[0]].m_height;
            ball.distance = (math.distance(ball.x, ball.y));
        }

        if (enemy.found) {
            enemy.x = robot.pixy.ccc.blocks[signature[1]].m_x; 
            enemy.y = robot.pixy.ccc.blocks[signature[1]].m_y;
            enemy.angle = math.countAngle(enemy.x, enemy.y);
            enemy.square = robot.pixy.ccc.blocks[signature[1]].m_width * robot.pixy.ccc.blocks[signature[1]].m_height;
            enemy.distance = (math.distance(enemy.x, enemy.y));
        }
        
        if (home.found) {
            home.x = robot.pixy.ccc.blocks[signature[2]].m_x; 
            home.y = robot.pixy.ccc.blocks[signature[2]].m_y;
            home.angle = math.countAngle(home.x, home.y);
            home.square = robot.pixy.ccc.blocks[signature[2]].m_width * robot.pixy.ccc.blocks[signature[2]].m_height;
            home.distance = (math.distance(home.x, home.y));
        }
    }
    
    current_angle = ball.angle;

    if (abs(current_angle) > 0.2 && abs(current_angle) < 3.15) {
        current_angle += math.radian(30) * math.sign(current_angle);
    }

    ball_catched = robot.checkHole();

    if (robot.buttonPressed(0))
        target = robot.degree;

    if (robot.buttonPressed(1))
        speed = 255;

    if (robot.buttonPressed(0) && robot.buttonPressed(1) && robot.setTimer(switch_update_timer, 5000)) {
        switched_goals = !switched_goals;
        switch_update_timer = millis();
    }
    
    robot.updateGyro();
    int err = target - robot.degree;
    int u = (int(err) - (int((err)) % 180) * 2);
    int err_old = err; 
    int led_coef;

    if (ball_catched) {
        current_angle = 0;
        u = (enemy.angle) / PI * 180 - math.sign(enemy.angle) * 15;
        if (u > 50) 
            u = 50;
        if (u < -50) {
            u = -50;
        }
    }

    if (!ball.found) {
        if (home.found) {
            current_angle = home.angle;
            if (home.distance > 80) {
                speed = 180;
            } else {
                speed = 0;
            }
        } else {
            speed = 0;
        }
    }

    robot.updateLed(robot.led_value);
    
    for (int i = 0; i < 24; ++i) {
        if (robot.led_value[i]) {
            ++led_counter;
        }
    }

    if (led_counter >= 2) {
        for (int i = 0; i < 24; ++i) {
            if (robot.led_value[i]) {
                alpha1 = robot.led_angle[i];
                robot.led_value[i] = false;
                i = 24;
            }
        }
        for (int i = 0; i < 24; ++i) {
            if (robot.led_value[i]) {
                alpha2 = robot.led_angle[i];
                robot.led_value[i] = false;
                i = 24;
            }
        }
    }
            
    if (led_counter >= 2) {
        if (!mirror_exist) {
            int led_coef;
            
            if (abs(alpha1 - alpha2) < PI) 
                led_coef = 1;
            else
                led_coef = 0;

            if (home.found) {
                line_angle = home.angle;
            } else {
                line_angle = (alpha1 + alpha2) / 2 + math.radian(180) * led_coef;
            }
            
            mirror_exist = true;
        }
    }

//    if (led_counter >= 1) {
//        for (int i = 0; i < 24; ++i) { 
//            if (robot.led_value[i]) {
//                double delta1 = (abs(robot.led_angle[i] - alpha1) > PI) ? 2 * PI - abs(robot.led_angle[i] - alpha1) : abs(robot.led_angle[i] - alpha1);
//                double delta2 = (abs(robot.led_angle[i] - alpha2) > PI) ? 2 * PI - abs(robot.led_angle[i] - alpha2) : abs(robot.led_angle[i] - alpha2);
//                if (delta1 > math.radian(0) || delta2 > math.radian(0)) {
//                    mirror_line_catched = true;
//                    i = 24;
//                }
//            }
//        }
//    }

    if (led_counter >= 1) {
        line_catched = true;
        mirror_line_catched = true;
    } else {
        line_catched = false;
    }

    if (line_catched) {
        line_timer = millis();
    }

    if (robot.setTimer(line_timer, 50)) {
        line_catched = false;
        mirror_exist = false;
        mirror_line_catched = false;
    }

    if (!mirror_line_catched) {
        timer = millis();
    }
    
    if (mirror_line_catched ) {
        if (robot.setTimer(timer, 800) && !line_catched) {
            mirror_line_catched = false;
        } else {
            current_angle = line_angle;
            speed = 110;
        }
    }
    
    for (int i = 0; i < 24; ++i) {
        robot.led_value[i] = false;
    }

    if (robot.setTimer(serial_timer, 300)) {
        String mail = "";
        mail = robot.createMail("enemy distance", enemy.distance, mail);
        mail = robot.createMail("line angle", (int(line_angle * 180 / PI)) % 360, mail);
        mail = robot.createMail("angle", (int(current_angle * 180 / PI)) % 360, mail);
        mail = robot.createMail("ball d", ball.distance, mail);
        mail = robot.createMail("u", u, mail);
        robot.sendMail(mail);
        serial_timer = millis();
    }
            
    Serial.print(ball.distance);
    Serial.print(' ');
    Serial.print(enemy.distance);
    Serial.print(' ');
    Serial.print(home.distance);
    Serial.print(' ');    
    Serial.println();
    
    robot.moveAngle(current_angle, speed, u);
}