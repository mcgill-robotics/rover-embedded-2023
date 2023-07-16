#include <Arduino.h>

#define MOT1_DIR 2
#define MOT1_PWM 3
#define MOT2_DIR 4
#define MOT2_PWM 5
#define DIR 7
#define STEP 8 
#define M1 9
#define M0 10
#define MOIST_1 17
#define MOIST_2 16
#define MOIST_3 15
#define MOIST_4 24
#define LIM_1 21
#define LIM_2 22

#define MOTOR_STEPS 200
#define RPM 75

#define STEPPER_FREQ 1000.0
#define GEAR_RATIO 3
#define TIME_ANGLE_CNST (STEPPER_FREQ/8)*1.8/1000000/3

void science_setup();
void science_loop();
    