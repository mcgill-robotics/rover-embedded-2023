//#ifdef SCIENCE //LEAVE THIS AT THE TOP OF THIS FILE

#include <Arduino.h>
#include "science.h"
#include "DRV8834.h"

#define MOTOR_STEPS 200
#define RPM 75

#define DIR 8
#define STEP 9
#define M0 10
#define M1 11

DRV8834 stepper(MOTOR_STEPS, DIR, STEP, M0, M1);

/*void writeToStepperMotor(int steps, int direction) {
  //If direction = 1 --> CW
  //If direction = -1 --> CCW
  stepper.rotate(steps*direction)*/

void science_setup () {
 // SerialUSB.begin(9600);
  //stepper.begin(RPM, 1);
  //stepper.enable();
  pinMode(DIR, OUTPUT); 
  pinMode(STEP, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);

  digitalWrite(M0, LOW);
  digitalWrite(M1, HIGH);
  digitalWrite(DIR, HIGH); 

}

void science_loop () {
  for (int i = 0; i < MOTOR_STEPS; i++){
    digitalWrite(STEP, HIGH);
    delay(5); 
    digitalWrite(STEP, LOW); 
    delay(5);
  }
}

//#endif //LEAVE THIS AT THE BOTTOM OF THIS FILE