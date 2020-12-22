#include "exist.h"

Robot robot;
Math math;

camera ball, home, enemy;
const double epsilon = pow(10, -6);

long long int timer = 0;
long long int line_timer = 0;
long long int delay_timer = 0;
long long int update_timer = 0;
long long int serial_timer = 0;
long long int kicker_timer = 0;
long long int ball_exist_timer = 0;
long long int ball_catched_timer = 0;
long long int switch_update_timer = 0;

bool mirror_exist = false;
bool switched_goals = false;
bool mirror_line_catched = false;

double alpha1 = -1.0;
double alpha2 = -1.0;
double line_angle = 0.0;
double current_angle = 0.0;

int target = 0;
int target_back = 0;
int target_const = 0;
int beta1, beta2, x_old, y_old;

void setup() {
    robot.init();
    robot.pixy.init();
    robot.updateGyro();
    target = robot.degree;
    target_back = target + 180;
    target_const = target;
}

void loop() {
    short speed = 240;
    short led_counter = 0;
    target = target_const;

    if (robot.buttonPressed(0)) {
        target_const = robot.degree;
    }

    if (robot.buttonPressed(1)) {
        //target_back = robot.degree;
    }

    if (robot.buttonPressed(0) && robot.buttonPressed(1) && robot.setTimer(switch_update_timer, 5000)) {
        switched_goals = !switched_goals;
        switch_update_timer = millis();
    }
    
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
            int length = robot.pixy.ccc.blocks[signature[1]].m_width;
            enemy.angle = math.countAngle(enemy.x, enemy.y - length / 4);
            enemy.square = robot.pixy.ccc.blocks[signature[1]].m_width * robot.pixy.ccc.blocks[signature[1]].m_height;
            enemy.distance = (math.distance(enemy.x, enemy.y));
        }
        
        if (home.found) {
            home.x = robot.pixy.ccc.blocks[signature[2]].m_x; 
            home.y = robot.pixy.ccc.blocks[signature[2]].m_y;
            int length = robot.pixy.ccc.blocks[signature[2]].m_width;
            home.angle = math.countAngle(home.x, home.y - length / 4);
            home.square = robot.pixy.ccc.blocks[signature[2]].m_width * robot.pixy.ccc.blocks[signature[2]].m_height;
            home.distance = (math.distance(home.x, home.y));
        }
    }

    int delta = abs(abs(enemy.x - 162) / cos(enemy.angle)) - abs(abs(home.x - 162) / cos(home.angle));
    
    robot.updateGyro();
    int err = target - robot.degree;
    int u = (int(err) - (int((err)) % 180) * 2);
    int err_old = err; 

    robot.updateLed(robot.led_value);
    
    for (int i = 0; i < 24; ++i) {
        if (robot.led_value[i]) {
            ++led_counter;
        }
        Serial.print(robot.led_value[i]);
        Serial.print(' ');
        Serial.println();
    }
    
    bool line_catched = led_counter >= 1;
    bool ball_catched = robot.checkHole();

    if (!ball_catched) {
        ball_catched_timer = millis();
    }
    
    if (ball.found || ball_catched) {
        ball_exist_timer = millis();
    }

    if (abs(ball.angle) > 0.2 && abs(ball.angle) < 3.15 && ball.distance < 85 && !ball_catched) {
        ball.angle += math.radian(60) * math.sign(current_angle);
    }
    
    if (ball.distance < 60 && abs(ball.angle) < 0.25 && !ball_catched) {
        speed = 140;
        if (abs(robot.degree - target) < 15) {
            u = ((ball.angle) / PI * 180) * 2;
        }
        if (abs(ball.angle) < 0.15) {
            ball.angle = 0.0;
        }
    }

    current_angle = ball.angle;

    double dt = 100.0;
    
    if (robot.setTimer(update_timer, dt) ){
        double vx = (ball.x - x_old) / dt;
        double vy = (ball.y - y_old) / dt;
        x_old = ball.x;
        y_old = ball.y;
//        speed = sqrt(pow((speed * cos(current_angle) + vy), 2) + pow((speed * sin(current_angle) + vx), 2));
    }

    if (robot.setTimer(ball_exist_timer, 150)) {
        if (home.found) {
            current_angle = home.angle;
            if (delta < 0) {
                speed = 240;
            } else {
                speed = 0;
            }
        } else {
            speed = 0;
        }
    }

    if (robot.setTimer(ball_catched_timer, 100)) {
        speed = 110;
        robot.runDribler(1400);
        if (robot.setTimer(ball_catched_timer, 400)) {
            robot.runDribler(1250);
            speed = 0;
            u = ((enemy.angle - PI * math.sign(enemy.angle)) / PI * 180) * 2 * 0.5;
            if (u > 45) {
               u  = 45;
            }
            if (u < -45) {
                u = -45;
            }
            current_angle = PI;
        }
        if (robot.setTimer(ball_catched_timer, 1600)) {
            speed = 120;
            robot.runDribler(1250);
            u = ((enemy.angle - PI * math.sign(enemy.angle)) / PI * 180) * 2;
            if (u > 45) {
               u  = 45;
            }
            if (u < -45) {
                u = -45;
            }
            current_angle = PI;
            if (delta < -5 && line_catched) {
                speed = 0;
                u = ((enemy.angle) / PI * 180) * 2;
                if (abs(u) < 5 && robot.setTimer(kicker_timer, 4500)) {
                   robot.hitBall();
                   kicker_timer = millis();
               }
            }
        }
    } else {
        robot.runDribler(700); 
    }

    if (led_counter >= 2) {
        int index;
        for (int i = 0; i < 24; ++i) {
            if (robot.led_value[i]) {
                alpha1 = robot.led_angle[i];
                robot.led_value[i] = false;
                index = i;
                i = 24;
            }
        }
        for (int i = 0; i < 24; ++i) {
            if (robot.led_value[i]) {
                alpha2 = robot.led_angle[i];
                robot.led_value[index] = true;
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

            if (home.found || enemy.found) {
                if (delta > 0) {
                    line_angle = enemy.angle + math.radian(25) * math.sign(enemy.angle);
                } else {
                    line_angle = home.angle - math.radian(25) * math.sign(home.angle);
                }
            } else {
                line_angle = (alpha1 + alpha2) / 2 + math.radian(180) * led_coef;
            }
            
            int k = 2;
            beta1 = int(alpha1 * 180 / PI / 15 + 12) % 24;
            beta2 = int(alpha2 * 180 / PI / 15 + 12) % 24;
            
            if (abs(beta1 - beta2) <= 12) {
                    max(beta1, beta2) = (max(beta1, beta2) + k) % 24;
                    min(beta1, beta2) = (min(beta1, beta2) - k + 24) % 24;
            } else {
                    max(beta1, beta2) = (max(beta1, beta2) - k + 24) % 24;
                    min(beta1, beta2) = (min(beta1, beta2) + k) % 24;
            }
            mirror_exist = true;
        }
    }

    if (led_counter >= 1) {
        for (int i = 0; i < 24; ++i) { 
            if (robot.led_value[i]) {
                if (abs(beta1 - beta2) <= 12) {
                    if (i <= max(beta1, beta2) && i >= min(beta1, beta2)) {
                        mirror_line_catched = true;
                        i = 24;
                    } else {
                        mirror_line_catched = false;
                    }
                } else {
                    if (i >= max(beta1, beta2) || i <= min(beta1, beta2)) {
                        mirror_line_catched = true;
                        i = 24;
                    } else {
                        mirror_line_catched = false;
                    }
                }
            }
        }
    }

    if (line_catched) {
        line_timer = millis();
        if (!ball_catched) {
            speed = 140;
        }
    }

    if (robot.setTimer(line_timer, 150)) {
        mirror_exist = false;
    }

    if (!mirror_line_catched) {
        timer = millis();
    }
    
    if (mirror_line_catched) {
        if (!robot.setTimer(timer, 100)) {
            current_angle = line_angle;
            speed = 170;
        } else {
            timer = millis();
        }
    }

    if (robot.setTimer(serial_timer, 200)) {
        String mail = "";
        mail = robot.createMail("delta", delta, mail);
        mail = robot.createMail("angle", (int(current_angle * 180 / PI)) % 360, mail);
        mail = robot.createMail("lin_angle", (int(line_angle * 180 / PI)) % 360, mail);
        mail = robot.createMail("enemy d", abs(abs(enemy.x - 162) / cos(enemy.angle)), mail);
        mail = robot.createMail("home d", abs(abs(home.x - 162) / cos(home.angle)), mail);
        robot.sendMail(mail);
        serial_timer = millis();
    }  
    
//    Serial.print(current_angle);
//    Serial.print(' ');
//    Serial.print(line_catched);
//    Serial.print(' ');  
//    Serial.print(beta1);
//    Serial.print(' ');
//    Serial.print(beta2);
//    Serial.print(' ');  
//    Serial.println();
    robot.moveAngle(current_angle, speed, u);
}
