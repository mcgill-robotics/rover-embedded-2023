#include <Arduino.h>
#include "Servo.h"

#define ANTENNA_PWM_PIN 1

extern float rover_coords[2]; //latitude, longitude
extern float antenna_positioning[3]; //latitude, longitude, compass angle

void antenna_setup();
void antenna_loop();