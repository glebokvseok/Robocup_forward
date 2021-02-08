#include "functions.h"
#include <avr/eeprom.h>

long long int timer = 0;

int maxi[24];
int mini[24];
long average_green[24][2];
long average_white[24][2];
int calibration_value[24];
double led_angle[24];

void setup() {
    Serial.begin(9600);
    pinMode(33, OUTPUT);
    pinMode(35, OUTPUT);
    pinMode(37, OUTPUT);
  
    digitalWrite(33, LOW);
    digitalWrite(35, LOW);
    digitalWrite(37, LOW);

    pinMode(22, INPUT_PULLUP);
    pinMode(23, INPUT_PULLUP);

    for (int i = 0; i < 24; ++i) {
        maxi[i] = 0;
        mini[i] = 10000;
        average_green[i][0] = 1000;
        average_white[i][0] = 1000;
        average_green[i][1] = 1;
        average_white[i][1] = 1;
    }   

    for (int j = 0; j < 4; ++j) { 
        for (int i = 0; i < 6; ++i) { 
            led_angle[j * 6 + i] = (j * 6 + i) * radian(15) - radian(30);
        }
    }
    led_angle[0] += radian(360);
    led_angle[1] += radian(360);
    
}

void loop() {
        for (int j = 0; j < 4; ++j) { // выбор номера мультиплексора от 0 до 3
                for (int i = 0; i < 6; ++i) { // перебор восьми ножек мультиплексора
                    if (readChannel(i, j) > 300) {
                        if (average_green[j * 6 + i][1] > 50 && readChannel(i, j) > average_green[j * 6 + i][0] / average_green[j * 6 + i][1] + 50) {
                            average_white[j * 6 + i][0] += readChannel(i, j);
                            ++average_white[j * 6 + i][1];
                        } else {
                            average_green[j * 6 + i][0] += readChannel(i, j);
                            ++average_green[j * 6 + i][1];
                        }
                    }
//                    Serial.print(readChannel(i, j));
//                    Serial.print(',');
//                    Serial.print(' ');
                }
            }
    
    for (int i = 0; i < 24; ++i) {
        calibration_value[i] = average_green[i][0] / average_green[i][1] + 0.3 * abs(average_white[i][0] / average_white[i][1] - average_green[i][0] / average_green[i][1]);
        Serial.print(calibration_value[i]);
        Serial.print(',');
        Serial.print(' ');
    }  
    
    bool left_button_pressed = digitalRead(23);
    bool right_button_pressed = digitalRead(22);

    if (!left_button_pressed) {
        for (int i = 0; i < 24; ++i) {
            maxi[i] = 0;
            mini[i] = 10000;
            average_green[i][0] = 1000;
            average_white[i][0] = 1000;
            average_green[i][1] = 1;
            average_white[i][1] = 1;
        }   
    }

    if (!right_button_pressed) {
         eeprom_write_block((void*)&calibration_value, 20, sizeof(calibration_value));
    }
    
    Serial.println(); 
}
