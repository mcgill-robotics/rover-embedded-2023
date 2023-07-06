#ifdef DRIVE //LEAVE THIS AT THE TOP OF THIS FILE

#include <Arduino.h>
#include "drive.h"
#include <arm_math.h>
#include "BLDCMotor.h"


WheelMotor LBMotor(LB_PWM_PIN, LB_HALL_A_PIN, LB_HALL_B_PIN, LB_HALL_C_PIN);
WheelMotor LFMotor(LF_PWM_PIN, LF_HALL_A_PIN, LF_HALL_B_PIN, LF_HALL_C_PIN);
WheelMotor RBMotor(RB_PWM_PIN, RB_HALL_A_PIN, RB_HALL_B_PIN, RB_HALL_C_PIN);
WheelMotor RFMotor(RF_PWM_PIN, RF_HALL_A_PIN, RF_HALL_B_PIN, RF_HALL_C_PIN);


const int sampling_period = (int) (1000000 / SAMPLING_FREQUENCY);
enum directions{FORWARDS=1, BACKWARDS=-1};
volatile bool ledOn = false;

unsigned long cycles = 0;

void takeReading();
void integrateReadings();
void printPercent(int i);

void drive_setup() {
  // put your setup code here, to run once:

  integrateReadings();

  LBMotor.writeSpeed(); 
  LFMotor.writeSpeed();
  RBMotor.writeSpeed(); 
  RFMotor.writeSpeed();

  Serial.println("Setup done");

  delay(SETUP_TIME_DELAY);

  ITimer.attachInterruptInterval(sampling_period, takeReading);
  attachAllInterrupts();
}

float speed_cmd = 1500;
void drive_loop() {
  // put your main code here, to run repeatedly:

  //Measure speed of all the motors
  integrateReadings();

  if(Serial.available()){
  speed_cmd = Serial.parseFloat(); 
  Serial.println(speed_cmd);
 }

  LBMotor.motor_us = LBMotor.update(speed_cmd);
  LBMotor.writeSpeed();
  cycles++; 
  //don't think we actually need this delay
  delay(10);
  //speed sweep: 

  for(int i = 1900; i >= 1100; i--){
    integrateReadings();
    LBMotor.motor_us = i;
    LBMotor.writeSpeed();
    printPercent(i);
    delay(10);
  }

  for(int i = 1100; i < 1500; i++){
    integrateReadings();
    LBMotor.motor_us = i;
    LBMotor.writeSpeed();
    printPercent(i);
    delay(10);
  }
}

void printPercent(int i){
  float percentage_sent = (i - 1500) / 4;
  Serial.print(percentage_sent);
  Serial.print(", ");
  Serial.println(LBMotor.real_speed);
}

void integrateReadings(){
  LBMotor.measureSpeed();
  LFMotor.measureSpeed();
  RBMotor.measureSpeed();
  RFMotor.measureSpeed();
}

/* -------------------------------------------------------------------------- */
/*                             Interrupt functions                            */
/* -------------------------------------------------------------------------- */

void attachAllInterrupts(){
  attachInterrupt(LB_HALL_A_PIN, lb_hall_a_int, CHANGE);
  attachInterrupt(LB_HALL_B_PIN, lb_hall_b_int, CHANGE);
  attachInterrupt(LB_HALL_C_PIN, lb_hall_c_int, CHANGE);

  attachInterrupt(LF_HALL_A_PIN, lf_hall_a_int, CHANGE);
  attachInterrupt(LF_HALL_B_PIN, lf_hall_b_int, CHANGE);
  attachInterrupt(LF_HALL_C_PIN, lf_hall_c_int, CHANGE);

  attachInterrupt(RB_HALL_A_PIN, rb_hall_a_int, CHANGE);
  attachInterrupt(RB_HALL_B_PIN, rb_hall_b_int, CHANGE);
  attachInterrupt(RB_HALL_C_PIN, rb_hall_c_int, CHANGE);

  attachInterrupt(RF_HALL_A_PIN, rf_hall_a_int, CHANGE);
  attachInterrupt(RF_HALL_B_PIN, rf_hall_b_int, CHANGE);
  attachInterrupt(RF_HALL_C_PIN, rf_hall_c_int, CHANGE);
}

void takeReading(){
  LFMotor.takeReading();
	LFMotor.takeReading2();
	LFMotor.takeReading3();
	LBMotor.takeReading();
	LBMotor.takeReading2();
	LBMotor.takeReading3();
	RFMotor.takeReading();
	RFMotor.takeReading2();
	RFMotor.takeReading3();
	RBMotor.takeReading();
	RBMotor.takeReading2();
	RBMotor.takeReading3();
}

void lb_hall_a_int(){
  LBMotor.readingA = digitalRead(LB_HALL_A_PIN);

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