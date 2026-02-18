#pragma once

#define SERVO_PIN_1 27
#define SERVO_PIN_2 19
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500

#define PIN_STEP 25
#define PIN_DIR  26
#define PIN_ENABLE 27
#define STEP_DELAY_US 160
//step per mm = [(step per rev x microstepping ) x gear ratio ] / lead type (T8)
const float STEPS_PER_MM = 400.0; //  Microstep 1/8 | ((200*8)*(2/1))8 = 400
