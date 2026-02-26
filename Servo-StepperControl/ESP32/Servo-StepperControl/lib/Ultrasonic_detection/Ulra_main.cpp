#include <Arduino.h>
#include "Ultra_detector.h"

void setup() {
    Serial.begin(115200);
    setupUltra(); // เรียกใช้การตั้งค่าจากไฟล์ .cpp
}

void loop() {
    loopUltra(); // เรียกใช้ loop ของเซนเซอร์
}