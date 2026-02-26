
#include "CtrlManager.h"
#include <Arduino.h>

void PlantingManager::ena_enable()  { digitalWrite(PIN_ENABLE, LOW); }
void PlantingManager::ena_disable() { digitalWrite(PIN_ENABLE, HIGH); }

long PlantingManager::mmToSteps(float mm) {
        return (long)(mm * STEPS_PER_MM);
    }
// trasnfer degree to Microseconds (500-2500)
int PlantingManager::angleToUs(float angle) {
    float pulse = 500.0 + (angle * (2000.0 / 180.0));
    return (int)(pulse + 0.5); // +0.5 (Rounding up)
}

void PlantingManager::begin() {
    // Setup Servos
    ESP32PWM::allocateTimer(0);
    servo_gripper.setPeriodHertz(50);
    servo_linear.setPeriodHertz(50);
    servo_plate.setPeriodHertz(50);
    servo_gripper.attach(SERVO_PIN_gripper, 500, 2500); // gripper
    servo_linear.attach(SERVO_PIN_linear, 500, 2500); // linear motion
    servo_plate.attach(SERVO_PIN_plate, 500, 2500);

    // Setup Stepper
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    ena_enable(); // เปิด Driver ทันที
    // _currentPosSteps = 0; // home position at 0
    // _targetPosSteps = 0;
    
    // set up Invert like before (TB6600)
    _stepper.setPinsInverted(true, true, false); 
    _stepper.setEnablePin(PIN_ENABLE); // use AccelStepper to manage ENA 
    
    _stepper.setMaxSpeed(1000.0);      // velocity of SUGGESTED_SPS 
    _stepper.setAcceleration(500.0); // Acceleration 
    _stepper.setMinPulseWidth(20);      // TB6600 Pulse width

    _stepper.setCurrentPosition(0); // home position at 0
    servo_plate.writeMicroseconds(0);
}

// void PlantingManager::runStepper() {
//     if (_currentPosSteps == _targetPosSteps) {
//         _isStepping = false;
//         return;
//     }

//     _isStepping = true;
//     unsigned long now = micros();
    
//     // use STEP_DELAY_US from config.h Ex. 500, 800)
//     if (now - _lastStepMicros >= STEP_DELAY_US) {
//         _lastStepMicros = now;

//         // define direction
//         if (_targetPosSteps > _currentPosSteps) {
//             digitalWrite(PIN_DIR, HIGH);
//             _currentPosSteps++;
//         } else {
//             digitalWrite(PIN_DIR, LOW);
//             _currentPosSteps--;
//         }

//         // create pulse (Bit-banging single pulse)
//         digitalWrite(PIN_STEP, HIGH);
//         delayMicroseconds(5); // small pulse to prevent blocking
//         digitalWrite(PIN_STEP, LOW);
//     }
// }

void PlantingManager::startPlantPattern() {
    if (_activeMode == IDLE) {
        _activeMode = PLANTING;
        _currentStep = 1;
        _previousMillis = millis();
    }
}

void PlantingManager::LoadPattern() {
    if (_activeMode == IDLE) {
        _activeMode = LOADING;
        _currentStep = 1;
        _previousMillis = millis();
    }
}

void PlantingManager::update() {
    // commands Stepper to process every steps (Non-blocking)
    _stepper.run();
    // runStepper();
    if (_activeMode == IDLE) return;
    // if (!_isPatternRunning) return;

    unsigned long currentMillis = millis();
    unsigned long elapsed = currentMillis - _previousMillis;

    // Debug Print each Step
    if (_currentStep != _lastPrintedStep) {
        Serial.print("Executing Step: "); Serial.println(_currentStep);
        _lastPrintedStep = _currentStep;
    }
    switch (_activeMode) {
        
        case PLANTING:
            switch (_currentStep) {
                case 1: // Reset positions & Move Stepper to idle
                    servo_gripper.write(158);
                    servo_linear.write(0);
                    _stepper.moveTo(mmToSteps(0)); // stepper go to 0 (starting point)
                    // wait until ___ms and Stepper reach it's destination
                    if ((elapsed >= 10000) || (elapsed >= 1000 && _stepper.distanceToGo() == 0)) { 
                        _currentStep++; _previousMillis = currentMillis; 
                    }
                    break;
                case 2: // Step 2: lead linear down
                    _stepper.moveTo(mmToSteps(200.0)); // stepper go to the point (250 mm)
                    if ((elapsed >= 10000) || (elapsed >= 1000 && _stepper.distanceToGo() == 0)) { 
                        _currentStep++; _previousMillis = currentMillis; 
                    }
                    break;
                case 3: // Step 2: servo linear down
                    servo_linear.write(180);
                    if (elapsed >= 3000) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 4: // Step 3: linear up
                    servo_linear.write(0);
                    if (elapsed >= 2500) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 5: // Step 4: gripper open, drop down plant
                    servo_gripper.write(20);
                    if (elapsed >= 3000) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 6: // 
                    servo_gripper.write(154);
                    if (elapsed >= 1000) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 7: // 
                    servo_linear.write(100);
                    if (elapsed >= 2500) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 8: // 
                    servo_gripper.write(150);
                    servo_linear.write(0);
                    if (elapsed >= 2500) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 9: // lead up for gripper to open
                    _stepper.moveTo(mmToSteps(190.0));
                    if ((elapsed >= 10000) || (elapsed >= 1000 && _stepper.distanceToGo() == 0)) { 
                        _currentStep++; _previousMillis = currentMillis; 
                    }
                    break;
                case 10: // gripper open to prepare digging
                    servo_gripper.write(20);
                    if (elapsed >= 1000) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 11: // 
                    _stepper.moveTo(mmToSteps(200.0)); // lead down to same height
                    if ((elapsed >= 10000) || (elapsed >= 1000 && _stepper.distanceToGo() == 0)) { 
                        _currentStep++; _previousMillis = currentMillis; 
                    }
                    break;
                case 12: // Step 4: linear down 
                    servo_linear.write(70);
                    if (elapsed >= 1500) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 13: // Step 4: gripper go beside plant 1/3
                    servo_gripper.write(60);
                    if (elapsed >= 1000) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 14: // Step 4: gripper go beside plant 2/3
                    servo_gripper.write(85);
                    if (elapsed >= 1000) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 15: // Step 4: gripper go beside plant 3/3
                    servo_gripper.write(125);
                    if (elapsed >= 1000) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                // case 9: 
                //     servo_gripper.write(60);
                //     if (elapsed >= 1500) { _currentStep++; _previousMillis = currentMillis; }
                //     break;
                // case 10: 
                //     servo_linear.write(0);
                //     if (elapsed >= 1500) { _currentStep++; _previousMillis = currentMillis; }
                //     break;
                // case 11: // Step 4: linear down 
                //     servo_linear.write(60);
                //     if (elapsed >= 1500) { _currentStep++; _previousMillis = currentMillis; }
                //     break;
                // case 12: // Step 4: gripper go beside plant 1/3
                //     servo_gripper.write(60);
                //     if (elapsed >= 1000) { _currentStep++; _previousMillis = currentMillis; }
                //     break;
                // case 13: // Step 4: gripper go beside plant 2/3
                //     servo_gripper.write(85);
                //     if (elapsed >= 1000) { _currentStep++; _previousMillis = currentMillis; }
                //     break;
                // case 14: // Step 4: gripper go beside plant 3/3
                    // servo_gripper.write(125);
                    // if (elapsed >= 2000) { _currentStep++; _previousMillis = currentMillis; }
                    // break;
                case 16: // Step 4: linear up 
                    servo_linear.write(0); 
                    // servo_gripper.write(100); 
                    if (elapsed >= 2000) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 17: // Step 4: gripper, stepper go idle
                    servo_gripper.write(158);
                    _stepper.moveTo(mmToSteps(0.0)); // stepper go to the point 
                    if ((elapsed >= 10000) || (elapsed >= 1000 && _stepper.distanceToGo() == 0)) { 
                        _currentStep++; _previousMillis = currentMillis; 
                    }
                    break;
                case 18: // จบ Pattern
                    // _isPatternRunning = false;
                    _currentStep = 0;
                    _activeMode = IDLE;
                    Serial.println("Plant Pattern Done!");
                    break;
            }
        case LOADING: 
            switch (_currentStep) {
                case 1: // Case 1 _servo_plate_CurrentAngle starts with 16.3 degrees
                    servo_plate.writeMicroseconds(angleToUs(HomeAngle)); // 16.3 degrees
                    if (elapsed >= 2000) { _currentStep++; _previousMillis = currentMillis; }

                    break;
                case 2: // ค่อยๆ ขยับทีละ 16.3 องศา (Smooth Move)
                // ตรวจสอบว่าถึงเวลาอัปเดตองศาหรือยัง (คล้าย delay(1) แต่ไม่ block)
                    if (currentMillis - _lastServoUpdate >= 15) { 
                        _lastServoUpdate = currentMillis;
                        if (_servo_plate_CurrentAngle < 16.3*2) {
                            _servo_plate_CurrentAngle += 0.8; // ความเร็วการขยับ
                            servo_plate.writeMicroseconds(angleToUs(_servo_plate_CurrentAngle));
                        } else {
                        if (elapsed >= 1000) { _currentStep++; _previousMillis = currentMillis; }
                        }
                    }
                    break;
                case 3:
                    servo_plate.writeMicroseconds(angleToUs(HomeAngle));
                    if (elapsed >= 1000) { _currentStep++; _previousMillis = currentMillis; }
                    _activeMode = IDLE;
                    Serial.println("Reset System Done!");
                    _currentStep = 0;
                    break;
            }
            break;
            
    }
}

void PlantingManager::stopAll() {
    _stepper.stop(); 
    _activeMode = IDLE;
    _currentStep = 0;
    Serial.println("SYSTEM STOPPED!");
}
