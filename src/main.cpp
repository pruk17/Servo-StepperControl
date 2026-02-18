#include <Arduino.h>
#include "CtrlManager.h"

PlantingManager robot;

void setup() {
    Serial.begin(115200);
    robot.begin();
}

void loop() {
    // 1. ตรวจสอบสถานะและขยับ  (ใช้ millis ไม่ขัดจังหวะ)
    robot.update();

    // 2. รับคำสั่งจาก Serial ได้ตลอดเวลา แม้กำลังขยับอยู่
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "1") {
            Serial.print("1 printed");
            robot.startPlantPattern();
        }
    }
}