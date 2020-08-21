#include <M5Stack.h>
#include "matrix.h"

void setup() {
    M5.begin();
    M5.Power.begin();
    Serial.begin(9600);
    Serial2.begin(9600);
    M5.Lcd.setFreeFont(&FreeSans18pt7b);
    M5.Lcd.setTextColor(GREEN, BLACK); // задать цвет для текста и фона
    M5.Lcd.drawString("Wake up, Neo", 30, 80);
    for (int j = 0; j < 10; ++j) {
        M5.Lcd.drawString(".", 250, 80);
        delay(100);
        M5.Lcd.drawString(".", 260, 80);
        delay(100);
        M5.Lcd.drawString(".", 270, 80);
        delay(100);
        M5.Lcd.drawString("       ", 250, 80);
        delay(100);
    }
    M5.Lcd.clear();
      for (int y = 0; y < 2500; y += 12) {
        for (int i = 0; i < 50; ++i) {
          M5.Lcd.drawString(multi[i], 0, y - i * 40);
        }
          M5.Lcd.clear();
      }
    M5.Lcd.drawString("Neo is finally awake", 10, 80);
    delay(300);
    M5.Lcd.clear();
    M5.Lcd.setFreeFont(&FreeSans12pt7b);
}

void loop() {
    int x = 0; 
    int y = 0;
    int counter = 0;
    int incoming = 0;
    char mail[100];
    String str = "";
    while(Serial2.available()) {
        incoming = Serial2.read();
        if (incoming == '!') {
            break;
        }
        mail[counter] = incoming;
        ++counter;
    }
    for (int i = 0; i < counter; ++i) {
        if (mail[i] == '|') {
            M5.Lcd.drawString(str, x, y);
            y += 30;
            str = "";
        } else {
            str += char(mail[i]);
        }
    }
    delay(100);
    M5.Lcd.clear();
}
