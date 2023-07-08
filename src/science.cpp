#ifdef SCIENCE //LEAVE THIS AT THE TOP OF THIS FILE

#include <Arduino.h>
#include "science.h"
#include "DRV8834.h"

#define MOTOR_STEPS 200

#define DIR 8
#define STEP 9
#define MS1 10
#define MS2 11
#define MS3 12
DRV8834 stepper(MOTOR_STEPS, DIR, STEP, MS1, MS2, MS3);

/*void writeToStepperMotor(int steps, int direction) {
  //If direction = 1 --> CW
  //If direction = -1 --> CCW
  stepper.rotate(steps*direction)*/

void science_setup () {
  SerialUSB.begin(9600);
  stepper.begin(30,1);
}

void science_loop () {
  SerialUSB.println("Upload");
  stepper.rotate(360);
}

#endif //LEAVE THIS AT THE BOTTOM OF THIS FILE