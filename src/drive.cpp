#ifdef DRIVE //LEAVE THIS AT THE TOP OF THIS FILE

#include "drive.h"

float targetSpeeds[4];
float realSpeeds[4];

WheelMotor LBMotor(LB_PWM_PIN, &targetSpeeds[0], &realSpeeds[0]);
WheelMotor LFMotor(LF_PWM_PIN, &targetSpeeds[1], &realSpeeds[1]);
WheelMotor RBMotor(RB_PWM_PIN, &targetSpeeds[2], &realSpeeds[2]);
WheelMotor RFMotor(RF_PWM_PIN, &targetSpeeds[3], &realSpeeds[3]);

void drive_setup() {

  delay(3000); //wait for motors to start up
}

void drive_loop() {

  updateControllers();

  writeSpeeds();
}

void writeSpeeds(){
  LFMotor.writeSpeed();
  LBMotor.writeSpeed();
  RFMotor.writeSpeed();
  RBMotor.writeSpeed();
}

void updateControllers(){
  LFMotor.update();
  LBMotor.update();
  RFMotor.update();
  RBMotor.update();
}


#endif //LEAVE THIS AT THE BOTTOM OF THIS FILE