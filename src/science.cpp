//#ifdef SCIENCE //LEAVE THIS AT THE TOP OF THIS FILE

#include <Arduino.h>
#include "science.h"
#include "DRV8834.h"
#include "AccelStepper.h"
#include "CytronMotorDriver.h"

float scienceTargets[3];
float moistures[4];
float angleFeedback = 0;

AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);
IntervalTimer stepperTimer;

CytronMD MOT1(PWM_DIR, MOT1_PWM, MOT1_DIR);
CytronMD MOT2(PWM_DIR, MOT2_PWM, MOT2_DIR);

void stepperISR(){
  stepper.run();
}

void readAllMoistures(){
  moistures[0] = analogRead(MOIST_1) * (100.0/4095.0);
  moistures[1] = analogRead(MOIST_2) * (100.0/4095.0);
  moistures[2] = analogRead(MOIST_3) * (100.0/4095.0);
  moistures[3] = analogRead(MOIST_4) * (100.0/4095.0);
  angleFeedback = stepper.currentPosition() / GEAR_RATIO;
}

void writeAllMotors(){
  stepper.move(scienceTargets[0] * GEAR_RATIO);
  MOT1.setSpeed(scienceTargets[1]);
  MOT2.setSpeed(scienceTargets[2]);
}

void science_setup () {
  SerialUSB.begin(115200);

  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);

  digitalWrite(M0, LOW);
  digitalWrite(M1, HIGH);

  analogWriteFrequency(STEP, STEPPER_FREQ);
  analogWriteResolution(12);

  // stepperTimer.begin(stepperISR, 1000000/STEPPER_FREQ);
  scienceTargets[0] = 45;
  stepper.setMaxSpeed(1000.0);
  stepper.setAcceleration(200.0);
  // stepper.setSpeed(250);
  // stepper.moveTo(100);

  stepper.moveTo(200.0);
  SerialUSB.println("Moving stepper");
}

void science_loop () {

  //readAllMoistures();
  if(stepper.distanceToGo() == 0){
    stepper.moveTo(-stepper.currentPosition());
  }
  // stepper.moveTo(150);

 // writeAllMotors();
 stepper.run();
}



//#endif //LEAVE THIS AT THE BOTTOM OF THIS FILE