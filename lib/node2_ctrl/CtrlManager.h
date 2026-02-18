#pragma once

#include <ESP32Servo.h>
#include <AccelStepper.h>
#include "config.h"

enum TaskMode { IDLE, PLANTING, LOADING };
class PlantingManager {
private:
    TaskMode _activeMode = IDLE;
    Servo _servo1, _servo2;
    AccelStepper _stepper;
    
    unsigned long _previousMillis = 0; // collect the lastest time to move
    int _currentStep = 0; // to check which Step in the Pattern
    bool _isPatternRunning = false; // to check if the program is running 
    int _lastPrintedStep = -1;
    // Helper for TB6600 (Active-Low)
    void ena_enable();
    void ena_disable();

public:
    PlantingManager() : _stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR) {}

    long mmToSteps(float StepPermm);
    void begin(); // command to start
    void startPlantPattern();
    void LoadPattern();
    void update(); // call in loop()
    void stopAll(); // for Emergency Stop
};


