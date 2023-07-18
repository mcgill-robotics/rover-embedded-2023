//#ifdef SCIENCE //LEAVE THIS AT THE TOP OF THIS FILE

#include <Arduino.h>
#include "science.h"
#include "DRV8834.h"
#include "AccelStepper.h"
#include "CytronMotorDriver.h"

float scienceTargets[2];
float moistures[4];

AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);
IntervalTimer stepperTimer;

CytronMD MOT1(PWM_DIR, MOT1_PWM, MOT1_DIR);
CytronMD MOT2(PWM_DIR, MOT2_PWM, MOT2_DIR);

void stepperISR(){
  stepper.run();
}

void science_setup () {
  SerialUSB.begin(115200);

  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);

  digitalWrite(M0, LOW);
  digitalWrite(M1, HIGH);

  analogWriteFrequency(STEP, STEPPER_FREQ);
  analogWriteResolution(12);

  stepperTimer.begin(stepperISR, 1000000/STEPPER_FREQ);

}

void science_loop () {

  

  readAllMoistures();

  writeAllMotors();
}

void readAllMoistures(){
  moistures[0] = analogRead(MOIST_1) * (100.0/4095.0);
  moistures[1] = analogRead(MOIST_2) * (100.0/4095.0);
  moistures[3] = analogRead(MOIST_3) * (100.0/4095.0);
  moistures[4] = analogRead(MOIST_4) * (100.0/4095.0);

}

void writeAllMotors(){
  stepper.moveTo(scienceTargets[0] * GEAR_RATIO);
  MOT1.setSpeed(scienceTargets[1]);
  MOT2.setSpeed(scienceTargets[2]);
}

//#endif //LEAVE THIS AT THE BOTTOM OF THIS FILE