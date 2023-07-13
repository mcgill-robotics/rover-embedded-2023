#ifdef DRIVE //LEAVE THIS AT THE TOP OF THIS FILE

#include <Arduino.h>
#include "drive.h"
#include <arm_math.h>
#include "BLDCMotor.h"

WheelMotor LBMotor(LB_PWM_PIN, LB_HALL_A_PIN, LB_HALL_B_PIN, LB_HALL_C_PIN);
WheelMotor LFMotor(LF_PWM_PIN, LF_HALL_A_PIN, LF_HALL_B_PIN, LF_HALL_C_PIN);
WheelMotor RBMotor(RB_PWM_PIN, RB_HALL_A_PIN, RB_HALL_B_PIN, RB_HALL_C_PIN);
WheelMotor RFMotor(RF_PWM_PIN, RF_HALL_A_PIN, RF_HALL_B_PIN, RF_HALL_C_PIN);

volatile int counterInt = 0;

IntervalTimer readingTimer;
// TeensyTimer ITimer(TEENSY_TIMER_1);
// Servo motor; 

void drive_setup() {

  SerialUSB.begin(115200);
  LBMotor.resetMotor(); 
  LBMotor.real_speed = 0;
  // LFMotor.resetMotor();
  // RBMotor.resetMotor(); 
  // RFMotor.resetMotor();

  // ITimer.attachInterruptInterval(1000000 / SAMPLING_FREQUENCY, takeReadings);
  // readingTimer.begin(takeReadings, 1000000 / SAMPLING_FREQUENCY);
  attachAllInterrupts();
}

int speed_cmd = 0;
void drive_loop() {
  // SerialUSB.println("Looping");
  // measureSpeeds();

  // updateControllers();

  // writeSpeeds();
  if(SerialUSB.available()){
    speed_cmd = SerialUSB.parseInt();
    SerialUSB.println(speed_cmd);
  }
  LBMotor.setTargetSpeed(speed_cmd);
  LBMotor.update();
  SerialUSB.print("Speed: ");
  SerialUSB.println(counterInt);
  SerialUSB.print("Motor_us: ");
  SerialUSB.println(LBMotor.motor_us);
  // LBMotor.real_speed = result;
  //LBMotor.real_speed = (int) (1500 + 4*speed_cmd);
  //LBMotor.motor_us = (int) (1500 + 4*speed_cmd);
  LBMotor.writeSpeed();
  delay(10);
  
  //Send feedback message
}

void measureSpeeds(){
  LBMotor.measureSpeed();
  // LFMotor.measureSpeed(ITimer);
  // RBMotor.measureSpeed(ITimer);
  // RFMotor.measureSpeed(ITimer);
}

void takeReadings(){
  LFMotor.takeReading();
	// LBMotor.takeReading();
	// RFMotor.takeReading();
	// RBMotor.takeReading();
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

void attachAllInterrupts(){
  attachInterrupt(digitalPinToInterrupt(LB_HALL_A_PIN), lb_hall_a_int, CHANGE);
  attachInterrupt(LB_HALL_B_PIN, lb_hall_b_int, CHANGE);
  attachInterrupt(LB_HALL_C_PIN, lb_hall_c_int, CHANGE);

  // attachInterrupt(LF_HALL_A_PIN, lf_hall_a_int, CHANGE);
  // attachInterrupt(LF_HALL_B_PIN, lf_hall_b_int, CHANGE);
  // attachInterrupt(LF_HALL_C_PIN, lf_hall_c_int, CHANGE);

  // attachInterrupt(RB_HALL_A_PIN, rb_hall_a_int, CHANGE);
  // attachInterrupt(RB_HALL_B_PIN, rb_hall_b_int, CHANGE);
  // attachInterrupt(RB_HALL_C_PIN, rb_hall_c_int, CHANGE);

  // attachInterrupt(RF_HALL_A_PIN, rf_hall_a_int, CHANGE);
  // attachInterrupt(RF_HALL_B_PIN, rf_hall_b_int, CHANGE);
  // attachInterrupt(RF_HALL_C_PIN, rf_hall_c_int, CHANGE);
}

/* -------------------------------------------------------------------------- */
/*                             Interrupt functions                            */
/* -------------------------------------------------------------------------- */

void lb_hall_a_int(){
  LBMotor.readingA = digitalRead(LB_HALL_A_PIN);
  counterInt++;

  if(LBMotor.readingA == 1){
    LBMotor.directionCounter += (LBMotor.currentAngle >= 180) ? BACKWARDS : FORWARDS;
    LBMotor.currentAngle = 0;
  }else{
    LBMotor.directionCounter += (LBMotor.currentAngle < 180) ? BACKWARDS : FORWARDS;
    LBMotor.currentAngle = 180;
  }
  LBMotor.directionCounter = (LBMotor.directionCounter > DIRECTIONBUFFER-1) ? DIRECTIONBUFFER : LBMotor.directionCounter;
  LBMotor.directionCounter = (LBMotor.directionCounter < DIRECTIONBUFFER * -1) ? DIRECTIONBUFFER * -1 : LBMotor.directionCounter;
}

void lb_hall_b_int(){
  LBMotor.readingB = digitalRead(LB_HALL_B_PIN);
  counterInt++;

  if(LBMotor.readingB == 1){
    LBMotor.directionCounter += (LBMotor.currentAngle < 120 || LBMotor.currentAngle == 300) ? BACKWARDS : FORWARDS;
    LBMotor.currentAngle = 120;
  }else{
    LBMotor.directionCounter += (LBMotor.currentAngle <= 60) ? FORWARDS : BACKWARDS;
    LBMotor.currentAngle = 300;
  }

  LBMotor.directionCounter = (LBMotor.directionCounter > DIRECTIONBUFFER-1) ? DIRECTIONBUFFER : LBMotor.directionCounter;
  LBMotor.directionCounter = (LBMotor.directionCounter < DIRECTIONBUFFER * -1) ? DIRECTIONBUFFER * -1 : LBMotor.directionCounter;
}

void lb_hall_c_int(){
  counterInt++;
  LBMotor.readingC = digitalRead(LB_HALL_C_PIN);

  if(LBMotor.readingC == 1){
    LBMotor.directionCounter += (LBMotor.currentAngle == 300 || LBMotor.currentAngle == 0) ? FORWARDS : BACKWARDS;
    LBMotor.currentAngle = 240;
  }else{
    LBMotor.directionCounter += (LBMotor.currentAngle >= 240 || LBMotor.currentAngle == 0) ? BACKWARDS : FORWARDS;
    LBMotor.currentAngle = 60;
  }

  LBMotor.directionCounter = (LBMotor.directionCounter > DIRECTIONBUFFER-1) ? DIRECTIONBUFFER : LBMotor.directionCounter;
  LBMotor.directionCounter = (LBMotor.directionCounter < DIRECTIONBUFFER * -1) ? DIRECTIONBUFFER * -1 : LBMotor.directionCounter;
}

void lf_hall_a_int(){
  LFMotor.readingA = digitalRead(LF_HALL_A_PIN);

  if(LFMotor.readingA == 1){
    LFMotor.directionCounter += (LFMotor.currentAngle >= 180) ? BACKWARDS : FORWARDS;
    LFMotor.currentAngle = 0;
  }else{
    LFMotor.directionCounter += (LFMotor.currentAngle < 180) ? BACKWARDS : FORWARDS;
    LFMotor.currentAngle = 180;
  }
  LFMotor.directionCounter = (LFMotor.directionCounter > DIRECTIONBUFFER-1) ? DIRECTIONBUFFER : LFMotor.directionCounter;
  LFMotor.directionCounter = (LFMotor.directionCounter < DIRECTIONBUFFER * -1) ? DIRECTIONBUFFER * -1 : LFMotor.directionCounter;
}

void lf_hall_b_int(){
  LFMotor.readingB = digitalRead(LF_HALL_B_PIN);

  if(LFMotor.readingB == 1){
    LFMotor.directionCounter += (LFMotor.currentAngle < 120 || LFMotor.currentAngle == 300) ? BACKWARDS : FORWARDS;
    LFMotor.currentAngle = 120;
  }else{
    LFMotor.directionCounter += (LFMotor.currentAngle <= 60) ? FORWARDS : BACKWARDS;
    LFMotor.currentAngle = 300;
  }

  LFMotor.directionCounter = (LFMotor.directionCounter > DIRECTIONBUFFER-1) ? DIRECTIONBUFFER : LFMotor.directionCounter;
  LFMotor.directionCounter = (LFMotor.directionCounter < DIRECTIONBUFFER * -1) ? DIRECTIONBUFFER * -1 : LFMotor.directionCounter;
}

void lf_hall_c_int(){
  LFMotor.readingC = digitalRead(LF_HALL_C_PIN);

  if(LFMotor.readingC == 1){
    LFMotor.directionCounter += (LFMotor.currentAngle == 300 || LFMotor.currentAngle == 0) ? FORWARDS : BACKWARDS;
    LFMotor.currentAngle = 240;
  }else{
    LFMotor.directionCounter += (LFMotor.currentAngle >= 240 || LFMotor.currentAngle == 0) ? BACKWARDS : FORWARDS;
    LFMotor.currentAngle = 60;
  }

  LFMotor.directionCounter = (LFMotor.directionCounter > DIRECTIONBUFFER-1) ? DIRECTIONBUFFER : LFMotor.directionCounter;
  LFMotor.directionCounter = (LFMotor.directionCounter < DIRECTIONBUFFER * -1) ? DIRECTIONBUFFER * -1 : LFMotor.directionCounter;
}

void rb_hall_a_int(){
  RBMotor.readingA = digitalRead(RB_HALL_A_PIN);

  if(RBMotor.readingA == 1){
    RBMotor.directionCounter += (RBMotor.currentAngle >= 180) ? BACKWARDS : FORWARDS;
    RBMotor.currentAngle = 0;
  }else{
    RBMotor.directionCounter += (RBMotor.currentAngle < 180) ? BACKWARDS : FORWARDS;
    RBMotor.currentAngle = 180;
  }
  RBMotor.directionCounter = (RBMotor.directionCounter > DIRECTIONBUFFER-1) ? DIRECTIONBUFFER : RBMotor.directionCounter;
  RBMotor.directionCounter = (RBMotor.directionCounter < DIRECTIONBUFFER * -1) ? DIRECTIONBUFFER * -1 : RBMotor.directionCounter;
}

void rb_hall_b_int(){
  RBMotor.readingB = digitalRead(RB_HALL_B_PIN);

  if(RBMotor.readingB == 1){
    RBMotor.directionCounter += (RBMotor.currentAngle < 120 || RBMotor.currentAngle == 300) ? BACKWARDS : FORWARDS;
    RBMotor.currentAngle = 120;
  }else{
    RBMotor.directionCounter += (RBMotor.currentAngle <= 60) ? FORWARDS : BACKWARDS;
    RBMotor.currentAngle = 300;
  }

  RBMotor.directionCounter = (RBMotor.directionCounter > DIRECTIONBUFFER-1) ? DIRECTIONBUFFER : RBMotor.directionCounter;
  RBMotor.directionCounter = (RBMotor.directionCounter < DIRECTIONBUFFER * -1) ? DIRECTIONBUFFER * -1 : RBMotor.directionCounter;
}

void rb_hall_c_int(){
  RBMotor.readingC = digitalRead(RB_HALL_C_PIN);

  if(RBMotor.readingC == 1){
    RBMotor.directionCounter += (RBMotor.currentAngle == 300 || RBMotor.currentAngle == 0) ? FORWARDS : BACKWARDS;
    RBMotor.currentAngle = 240;
  }else{
    RBMotor.directionCounter += (RBMotor.currentAngle >= 240 || RBMotor.currentAngle == 0) ? BACKWARDS : FORWARDS;
    RBMotor.currentAngle = 60;
  }

  RBMotor.directionCounter = (RBMotor.directionCounter > DIRECTIONBUFFER-1) ? DIRECTIONBUFFER : RBMotor.directionCounter;
  RBMotor.directionCounter = (RBMotor.directionCounter < DIRECTIONBUFFER * -1) ? DIRECTIONBUFFER * -1 : RBMotor.directionCounter;
}

void rf_hall_a_int(){
  RFMotor.readingA = digitalRead(RF_HALL_A_PIN);

  if(RFMotor.readingA == 1){
    RFMotor.directionCounter += (RFMotor.currentAngle >= 180) ? BACKWARDS : FORWARDS;
    RFMotor.currentAngle = 0;
  }else{
    RFMotor.directionCounter += (RFMotor.currentAngle < 180) ? BACKWARDS : FORWARDS;
    RFMotor.currentAngle = 180;
  }
  RFMotor.directionCounter = (RFMotor.directionCounter > DIRECTIONBUFFER-1) ? DIRECTIONBUFFER : RFMotor.directionCounter;
  RFMotor.directionCounter = (RFMotor.directionCounter < DIRECTIONBUFFER * -1) ? DIRECTIONBUFFER * -1 : RFMotor.directionCounter;
}

void rf_hall_b_int(){
  RFMotor.readingB = digitalRead(RF_HALL_B_PIN);

  if(RFMotor.readingB == 1){
    RFMotor.directionCounter += (RFMotor.currentAngle < 120 || RFMotor.currentAngle == 300) ? BACKWARDS : FORWARDS;
    RFMotor.currentAngle = 120;
  }else{
    RFMotor.directionCounter += (RFMotor.currentAngle <= 60) ? FORWARDS : BACKWARDS;
    RFMotor.currentAngle = 300;
  }

  RFMotor.directionCounter = (RFMotor.directionCounter > DIRECTIONBUFFER-1) ? DIRECTIONBUFFER : RFMotor.directionCounter;
  RFMotor.directionCounter = (RFMotor.directionCounter < DIRECTIONBUFFER * -1) ? DIRECTIONBUFFER * -1 : RFMotor.directionCounter;
}

void rf_hall_c_int(){
  RFMotor.readingC = digitalRead(RF_HALL_C_PIN);

  if(RFMotor.readingC == 1){
    RFMotor.directionCounter += (RFMotor.currentAngle == 300 || RFMotor.currentAngle == 0) ? FORWARDS : BACKWARDS;
    RFMotor.currentAngle = 240;
  }else{
    RFMotor.directionCounter += (RFMotor.currentAngle >= 240 || RFMotor.currentAngle == 0) ? BACKWARDS : FORWARDS;
    RFMotor.currentAngle = 60;
  }

  RFMotor.directionCounter = (RFMotor.directionCounter > DIRECTIONBUFFER-1) ? DIRECTIONBUFFER : RFMotor.directionCounter;
  RFMotor.directionCounter = (RFMotor.directionCounter < DIRECTIONBUFFER * -1) ? DIRECTIONBUFFER * -1 : RFMotor.directionCounter;
}


#endif //LEAVE THIS AT THE BOTTOM OF THIS FILE