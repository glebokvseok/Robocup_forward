#include "3xist.h"

Robot robot;
Math math;

const double epsilon = pow(10, -6);

long long int timer = 0;
long long int line_timer = 0;
long long int switch_update_timer = 0;

bool alpha_update = true;
bool line_catched = false;
bool mirror_exist = false;
bool switched_goals = false;
bool mirror_line_catched = false;

double alpha = -1.0;
double alpha1 = -1.0;
double alpha2 = -1.0;
double line_angle = 0;
double current_angle = 0;
double alpha1_mir = -1.0;
double alpha2_mir = -1.0;

int target = 0;
short speed = 120;
short counter = 0;

void setup() {
    robot.init();
    robot.pixy.init();
    robot.updateGyro();
    target = robot.degree;
}

void loop() {
    speed = 210;
    
    camera ball, home, enemy;
    
    robot.pixy.ccc.getBlocks();

    if (robot.pixy.ccc.numBlocks) {
        int signature[3] = {-1, -1, -1};
        
        robot.updateCamera(signature, robot.pixy.ccc.numBlocks);
        
        if (switched_goals) {
            int tmp = signature[1];
            signature[1] = signature[2];
            signature[2] = tmp;
        }

        enemy.distance = 0;
        home.distance = 0;
        
        ball.found = (signature[0] != -1);
        enemy.found = (signature[1] != -1);
        home.found = (signature[2] != -1);

        if (ball.found) {
            ball.x = robot.pixy.ccc.blocks[signature[0]].m_x; 
            ball.y = robot.pixy.ccc.blocks[signature[0]].m_y;
            ball.angle = -math.countAngle(ball.x, ball.y);
            ball.square = robot.pixy.ccc.blocks[signature[0]].m_width * robot.pixy.ccc.blocks[signature[0]].m_height;
            ball.distance = math.countDistance(math.distance(ball.x, ball.y));
        }

        if (enemy.found) {
            enemy.x = robot.pixy.ccc.blocks[signature[1]].m_x; 
            enemy.y = robot.pixy.ccc.blocks[signature[1]].m_y;
            enemy.angle = -math.countAngle(enemy.x, enemy.y);
            enemy.square = robot.pixy.ccc.blocks[signature[1]].m_width * robot.pixy.ccc.blocks[signature[1]].m_height;
            enemy.distance = math.countDistance(math.distance(enemy.x, enemy.y));
        }
        
        if (home.found) {
            home.x = robot.pixy.ccc.blocks[signature[2]].m_x; 
            home.y = robot.pixy.ccc.blocks[signature[2]].m_y;
            home.angle = -math.countAngle(home.x, home.y);
            home.square = robot.pixy.ccc.blocks[signature[2]].m_width * robot.pixy.ccc.blocks[signature[2]].m_height;
            home.distance = math.countDistance(math.distance(home.x, home.y));
        }
    }

    current_angle = ball.angle;

    if (abs(current_angle) > 0.3 && abs(current_angle) < 3.15 && ball.distance < 160)
        current_angle += math.radian(50) * math.sign(current_angle);

    if (robot.buttonPressed(0))
        target = robot.degree;

    if (robot.buttonPressed(1))
        speed = 210;

    if (robot.buttonPressed(0) && robot.buttonPressed(1) && robot.setTimer(switch_update_timer, 5000)) {
        switched_goals = !switched_goals;
        switch_update_timer = millis();
    }

    robot.updateGyro();
    int err = target - robot.degree;
    int u = (int(err) - (int((err)) % 180) * 2);
    int err_old = err; 
    int led_coef;

    if (!ball.found && home.found ) {
        current_angle = home.angle;
        if (home.distance > 500) {
            speed = 180;
        } else {
            speed = 0;
        }
    }

    robot.updateLed(robot.led_value);
    
    for (int i = 0; i < 24; ++i) {
        if (robot.led_value[i]) {
            ++counter;
        }
    }

    if (counter >= 1) {
        for (int i = 0; i < 24; ++i) {
            if (robot.led_value[i]) {
                alpha = robot.led_angle[i];
                i = 24;
            }
        }
    }

    if (counter >= 2) {
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
            
    if (counter >= 2) {
        if (!mirror_exist) {
            int led_coef;
            
            if (abs(alpha1 - alpha2) < PI) 
                led_coef = 1;
            else
                led_coef = 0;
                
            if (home.found && enemy.found) {
                line_angle = (enemy.distance > home.distance) ? enemy.angle : home.angle;
            } else if (home.found) {
                line_angle = home.angle;
            } else if (enemy.found) {
                line_angle = home.angle;
            } else {
                line_angle = (alpha1 + alpha2) / 2 + math.radian(180) * led_coef;
            }
            alpha1_mir = (alpha1 >= PI) ? alpha1 - PI : alpha1 + PI;
            alpha2_mir = (alpha2 >= PI) ? alpha2 - PI : alpha2 + PI;
            mirror_exist = true;
        }
    }

    if ((abs(-1.0 - alpha) > epsilon) && (alpha >= min(alpha1_mir, alpha2_mir)) && (alpha <= max(alpha1, alpha2)) && mirror_exist) {
        mirror_line_catched = true;
    }

    if ((abs(-1.0 - alpha) > epsilon)) {
        line_catched = true;
    } else {
        line_catched = false;
    }

    if (line_catched) {
        line_timer = millis();
        speed = 100;
    }

    if (robot.setTimer(line_timer, 400)) {
        alpha = -1.0;
        alpha1 = -1.0;
        alpha2 = -1.0;
        line_catched = false;
        mirror_exist = false;
        mirror_line_catched = false;
    }

    if (!mirror_line_catched) 
        timer = millis();
    
    if (mirror_line_catched) {
        if (robot.setTimer(timer, 400)) {
            alpha = -1.0;
            alpha1 = -1.0;
            alpha2 = -1.0;
            line_catched = false;
            mirror_exist = false;
            mirror_line_catched = false;
        } else {
            current_angle = line_angle;
            speed = 100;
        }
    }

    for (int i = 0; i < 24; ++i) {
        robot.led_value[i] = false;
    }

//    if (enemy.distance  < 500)
//        u = ((enemy.angle) + math.sign(enemy.angle) * math.radian(45)) * 10;

    Serial.print(line_catched);
    Serial.print(' ');
    Serial.print(mirror_line_catched);
    Serial.print(' ');
    Serial.print(current_angle);
    Serial.print(' ');   
    Serial.print(line_angle);
    Serial.print(' ');
    Serial.print(home.angle);
    Serial.print(' ');
    Serial.print(enemy.angle);
    Serial.print(' ');   
    Serial.println();
    
    robot.moveAngle(current_angle, speed, u);
}