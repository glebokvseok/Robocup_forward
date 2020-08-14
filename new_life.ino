#include "3xist.h"

Robot robot;
Math math;

const double epsilon = pow(10, -6);

long long int timer = 0;
long long int alpha_update_timer = 0;
long long int switch_update_timer = 0;

bool line_catched = false;
bool alpha_update = true;
bool switched_goals = false;

double line_angle = 0;
double alpha1 = -1.0;
double alpha2 = -1.0;

double current_angle = 0;
short speed = 120;
int target;

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

    if (robot.buttonPressed(0) && robot.buttonPressed(1) && robot.setTimer(switch_update_timer, 5000)) {
        switched_goals = !switched_goals;
        switch_update_timer = millis();
    }

    robot.updateGyro();
    int err = target - robot.degree;
    int u = (int(err) - (int((err)) % 180) * 2);
    int err_old = err; 
    int led_coef;

    if (!(abs(-1.0 - alpha1) > epsilon) && alpha_update) {
        alpha_update_timer = millis();
        alpha_update = true;
    } else {
        alpha_update = robot.setTimer(alpha_update_timer, 200);
    }
    
    if (alpha_update) 
        alpha1 = robot.updateLed();
    
    if ((abs(-1.0 - robot.updateLed()) > epsilon) && (abs(alpha1 - robot.updateLed()) > epsilon) && (abs(-1.0 - alpha1) > epsilon)) 
            alpha2 = robot.updateLed();
        else
            alpha2 = -1.0;
            
    if ((abs(-1.0 - alpha1) > epsilon) && (abs(-1.0 - alpha2) > epsilon)) {
        int led_coef;
        
        if (abs(alpha1 - alpha2) < PI) 
            led_coef = 1;
        else
            led_coef = 0;
            
        if (home.found && enemy.found) {
            line_angle = (enemy.distance > home.distance) ? enemy.angle : home.angle;
        } else if (home.found) {
            line_angle = home.angle;
        } else {
            line_angle = (alpha1 + alpha2) / 2 + math.radian(180) * led_coef;
        }
            line_catched = true;
    } 
    
    if (!ball.found && home.found ) {
        current_angle = home.angle;
        if (home.distance > 500) {
            speed = 180;
        } else {
            speed = 0;
        }
    }

    if (!line_catched) 
        timer = millis();
    
    if (line_catched) {
        if (robot.setTimer(timer, 600)) {
            alpha1 = -1.0;
            alpha2 = -1.0;
            line_catched = false;
        } else {
            current_angle = line_angle; 
        }
    }

//    if (enemy.distance  < 500)
//        u = ((enemy.angle) + math.sign(enemy.angle) * math.radian(45)) * 10;

    Serial.print(target);
    Serial.print(' ');
    Serial.print(robot.degree);
    Serial.print(' ');
    Serial.print(u);
    Serial.print(' ');
    Serial.print(robot.buttonPressed(1));
    Serial.print(' ');    
    Serial.println();
    
    robot.moveAngle(current_angle, speed, u);
}