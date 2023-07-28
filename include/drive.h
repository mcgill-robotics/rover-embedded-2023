#include <Arduino.h>
#include "BLDCMotor.h"

#define LB_PWM_PIN 5
#define LF_PWM_PIN 3
#define RB_PWM_PIN 4
#define RF_PWM_PIN 2


extern float targetSpeeds[4];
extern float realSpeeds[4];

extern float targetSpeeds[4];
extern float realSpeeds[4];

void drive_setup();
void drive_loop();

void updateControllers();
void writeSpeeds();