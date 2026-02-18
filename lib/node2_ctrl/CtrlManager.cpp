
#include "CtrlManager.h"
#include <Arduino.h>

void PlantingManager::ena_enable()  { digitalWrite(PIN_ENABLE, LOW); }
void PlantingManager::ena_disable() { digitalWrite(PIN_ENABLE, HIGH); }

long PlantingManager::mmToSteps(float mm) {
        return (long)(mm * STEPS_PER_MM);
    }

void PlantingManager::begin() {
    // Setup Servos
    ESP32PWM::allocateTimer(0);
    _servo1.setPeriodHertz(50);
    _servo2.setPeriodHertz(50);
    _servo1.attach(SERVO_PIN_1, 500, 2500);
    _servo2.attach(SERVO_PIN_2, 500, 2500);

    // Setup Stepper
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    ena_enable(); // เปิด Driver ทันที
    
    // set up Invert like before (TB6600)
    _stepper.setPinsInverted(true, true, false); 
    _stepper.setEnablePin(PIN_ENABLE); // use AccelStepper to manage ENA 
    
    _stepper.setMaxSpeed(1000.0);      // velocity of SUGGESTED_SPS 
    _stepper.setAcceleration(500.0); // Acceleration 
    _stepper.setMinPulseWidth(20);      // TB6600 Pulse width
}

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
    if (_activeMode == IDLE) return;
    if (!_isPatternRunning) return;

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
                    _servo1.write(158);
                    _servo2.write(0);
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
                    _servo2.write(180);
                    if (elapsed >= 3000) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 4: // Step 3: linear up
                    _servo2.write(0);
                    if (elapsed >= 2500) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 5: // Step 4: gripper open, drop down plant
                    _servo1.write(20);
                    if (elapsed >= 3000) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 6: // 
                    _servo1.write(154);
                    if (elapsed >= 1000) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 7: // 
                    _servo2.write(100);
                    if (elapsed >= 2500) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 8: // 
                    _servo1.write(150);
                    _servo2.write(0);
                    if (elapsed >= 2500) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 9: // lead up for gripper to open
                    _stepper.moveTo(mmToSteps(190.0));
                    if ((elapsed >= 10000) || (elapsed >= 1000 && _stepper.distanceToGo() == 0)) { 
                        _currentStep++; _previousMillis = currentMillis; 
                    }
                    break;
                case 10: // gripper open to prepare digging
                    _servo1.write(20);
                    if (elapsed >= 1000) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 11: // 
                    _stepper.moveTo(mmToSteps(200.0)); // lead down to same height
                    if ((elapsed >= 10000) || (elapsed >= 1000 && _stepper.distanceToGo() == 0)) { 
                        _currentStep++; _previousMillis = currentMillis; 
                    }
                    break;
                case 12: // Step 4: linear down 
                    _servo2.write(70);
                    if (elapsed >= 1500) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 13: // Step 4: gripper go beside plant 1/3
                    _servo1.write(60);
                    if (elapsed >= 1000) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 14: // Step 4: gripper go beside plant 2/3
                    _servo1.write(85);
                    if (elapsed >= 1000) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 15: // Step 4: gripper go beside plant 3/3
                    _servo1.write(125);
                    if (elapsed >= 1000) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                // case 9: 
                //     _servo1.write(60);
                //     if (elapsed >= 1500) { _currentStep++; _previousMillis = currentMillis; }
                //     break;
                // case 10: 
                //     _servo2.write(0);
                //     if (elapsed >= 1500) { _currentStep++; _previousMillis = currentMillis; }
                //     break;
                // case 11: // Step 4: linear down 
                //     _servo2.write(60);
                //     if (elapsed >= 1500) { _currentStep++; _previousMillis = currentMillis; }
                //     break;
                // case 12: // Step 4: gripper go beside plant 1/3
                //     _servo1.write(60);
                //     if (elapsed >= 1000) { _currentStep++; _previousMillis = currentMillis; }
                //     break;
                // case 13: // Step 4: gripper go beside plant 2/3
                //     _servo1.write(85);
                //     if (elapsed >= 1000) { _currentStep++; _previousMillis = currentMillis; }
                //     break;
                // case 14: // Step 4: gripper go beside plant 3/3
                    // _servo1.write(125);
                    // if (elapsed >= 2000) { _currentStep++; _previousMillis = currentMillis; }
                    // break;
                case 16: // Step 4: linear up 
                    _servo2.write(0); 
                    // _servo1.write(100); 
                    if (elapsed >= 2000) { _currentStep++; _previousMillis = currentMillis; }
                    break;
                case 17: // Step 4: gripper, stepper go idle
                    _servo1.write(158);
                    _stepper.moveTo(mmToSteps(0.0)); // stepper go to the point 
                    if ((elapsed >= 10000) || (elapsed >= 1000 && _stepper.distanceToGo() == 0)) { 
                        _currentStep++; _previousMillis = currentMillis; 
                    }
                    break;
                case 18: // จบ Pattern
                    _isPatternRunning = false;
                    _currentStep = 0;
                    Serial.println("Plant Pattern Done!");
                    break;
            }
        case LOADING: 
            switch (_currentStep) {
                case 1: // Case 1 ของตัวเอง
                    _servo1.write(158);
                    _servo2.write(0);
                    _stepper.moveTo(mmToSteps(0));
                    if (_stepper.distanceToGo() == 0) {
                        _currentStep++;
                        _previousMillis = currentMillis;
                    }
                    break;
                case 2:
                    Serial.println("Reset System Done!");
                    _activeMode = IDLE;
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
