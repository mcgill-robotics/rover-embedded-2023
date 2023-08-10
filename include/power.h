#include <Arduino.h>
#include "PWMServo.h"

#define PWM_Servo_1_Pin 7
#define PWM_Servo_2_Pin 8
#define Arm_24V_Pin 9
#define Arm_12V_Pin 10
#define Drive_Pin 11
#define Science_Pin 12
#define Drive_Curr_1_Pin 14
#define Drive_Curr_2_Pin 15
#define Drive_Curr_3_Pin 16
#define Drive_Curr_4_Pin 17
#define armBrushed_Curr_Pin 18
#define armBrushless_Curr_Pin 19
#define Sci5_Curr_Pin 20
#define Sci12_Curr_Pin 21

extern float currentsPower[8];
extern float powerAngles[2];
extern int powerRelays[4];

void power_setup();
void power_loop();

void readAllCurrents();
void moveServos();
void writeRelays();