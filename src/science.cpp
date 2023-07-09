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

void writeToStepperMotor(int steps, int direction) {
  //If direction = 1 --> CW
  //If direction = -1 --> CCW
  stepper.move(direction*steps);
}

void science_setup () {
  //SerialUSB.begin(9600);
  stepper.begin(RPM);
  //stepper.setEnableActiveState(LOW);
  stepper.enable();

  /*pinMode(DIR, OUTPUT); 
  pinMode(STEP, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  
  digitalWrite(M0, LOW);
  digitalWrite(M1, HIGH);
  digitalWrite(DIR, HIGH); */

}

void science_loop () {
  //writeToStepperMotor(1800,1);
  stepper.setMicrostep(1);

  stepper.rotate(-720);     // forward revolution
  //stepper.rotate(-60);

  //stepper.move(MOTOR_STEPS);    // forward revolution
  //stepper.move(-MOTOR_STEPS);
  //writeToStepperMotor(200,-1);
  //SerialUSB.println("2");
  //delay(1000);
  /*digitalWrite(STEP, HIGH);
  delay(5); 
  digitalWrite(STEP, LOW); 
  delay(5);*/
  delay(100);
}

//#endif //LEAVE THIS AT THE BOTTOM OF THIS FILE