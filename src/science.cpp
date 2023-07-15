//#ifdef SCIENCE //LEAVE THIS AT THE TOP OF THIS FILE

#include <Arduino.h>
#include "science.h"
#include "DRV8834.h"
#include "AccelStepper.h"

#define MOTOR_STEPS 200
#define RPM 75

// //TEST BREADBOARD
// #define DIR 8
// #define STEP 9
// #define M0 10
// #define M1 11

// ACTUAL BOARD
#define DIR 7
#define STEP 8
#define M1 9
#define M0 10

#define STEPPER_FREQ 1000

// DRV8834 stepper(MOTOR_STEPS, DIR, STEP, M0, M1);
IntervalTimer stepperTimer;

void stepperISR(){
  
}

void writeToStepperMotor(int steps, int direction) {
  //If direction = 1 --> CW
  //If direction = -1 --> CCW
  // stepper.move(direction*steps);
  
}

void science_setup () {
  //SerialUSB.begin(9600);
  // stepper.setMaxSpeed(200);
  // stepper.setSpeed(100);


  //stepper.setEnableActiveState(LOW);
  pinMode(DIR, OUTPUT); 
  pinMode(STEP, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  
  digitalWrite(M0, LOW);
  digitalWrite(M1, HIGH);
  digitalWrite(DIR, HIGH); 
  
  analogWriteFrequency(STEP, STEPPER_FREQ);
  analogWriteResolution(12);

  // stepperTimer.begin()

}

void science_loop () {
  //writeToStepperMotor(1800,1);
  // stepper.setMicrostep(1);

  analogWrite(STEP, 2048);
  delay(900);
  analogWrite(STEP, 0);
  delay(400);

  

  // stepper.rotate(-720);     // forward revolution
  //stepper.rotate(-60);

  // stepper.move(MOTOR_STEPS);    // forward revolution
  //stepper.move(-MOTOR_STEPS);
  //writeToStepperMotor(200,-1);
  // SerialUSB.println("science loop");
  //delay(1000);
  /*digitalWrite(STEP, HIGH);
  delay(5); 
  digitalWrite(STEP, LOW); 
  delay(5);*/
  // stepper.run();
}

//#endif //LEAVE THIS AT THE BOTTOM OF THIS FILE