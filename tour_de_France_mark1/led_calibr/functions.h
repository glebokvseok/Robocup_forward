#ifndef functions_h
#define functions_h

bool setTimer(long long timer, int dt) {
    return(millis() - timer > dt);
}

double radian(double angle) {
    return angle * PI / 180;
}

int readChannel(int n, int m) {
    int control_pins[3] = {33, 35, 37};
    int signal[4] = {A3, A5, A7, A1}; 
    int channels[6][3] = { // номера ножек мультиплексора в двоичном коде 
      {0, 0, 0},
      {1, 0, 0},
      {0, 1, 0},
      {1, 1, 0},
      {0, 0, 1},
      {1, 0, 1}
    };
    
    for (int i = 0; i < 3; i ++) {
        digitalWrite(control_pins[i], channels[n][i]);
    }
    int value = analogRead(signal[m]);
    return value;
}

#endif
