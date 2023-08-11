#include <Arduino.h>
#include "Servo.h"

#define ANTENNA_PWM_PIN 1

extern float rover_coords[2]; //latitude, longitude
extern float antenna_heading_params[4]; //latitude, longitude, compass angle
extern float servo_angle[1];

void antenna_setup();
void antenna_loop();