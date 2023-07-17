//#ifdef SCIENCE //LEAVE THIS AT THE TOP OF THIS FILE

#include <Arduino.h>
#include "science.h"
#include "DRV8834.h"
#include "AccelStepper.h"
#include "CytronMotorDriver.h"

float scienceTargets[2];

// //TEST BREADBOARD
// #define DIR 8
// #define STEP 9
// #define M0 10
// #define M1 11

// ACTUAL BOARD
// #define DIR 7
// #define STEP 8
// #define M1 9
// #define M0 10

// #define STEPPER_FREQ 1000.0
// #define GEAR_RATIO 3
// #define TIME_ANGLE_CNST (STEPPER_FREQ/8)*1.8/1000000/3

// DRV8834 stepper(MOTOR_STEPS, DIR, STEP, M0, M1);

//Create an IntervalTimer object (only a max of 4 can be active simultaneously)
AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);
IntervalTimer stepperTimer;

CytronMD MOT1(PWM_DIR, MOT1_PWM, MOT1_DIR);

void stepperISR(){
  analogWrite(STEP, 0);
  stepperTimer.end();
}

void writeToStepperMotor(int steps, int direction) {
  //If direction = 1 --> CW
  //If direction = -1 --> CCW
  // stepper.move(direction*steps);
  
}

void setUpMotor(){
  pinMode(MOT1_DIR, OUTPUT); 
  pinMode(MOT1_PWM, OUTPUT);
}


void setSpeed(int speed, int pwmpin, int dirpin){
  (speed < 0) ? digitalWrite(dirpin, LOW) : digitalWrite(dirpin, HIGH); 
  int motor_speed = abs(speed*2.55); 
  if(motor_speed > 255 || motor_speed < 255) motor_speed = 255;
  analogWrite(pwmpin, motor_speed);
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

  //setUpMotor(); 
  pinMode(MOT1_DIR, OUTPUT); 
  pinMode(MOT1_PWM, OUTPUT);


  digitalWrite(M0, LOW);
  digitalWrite(M1, HIGH);
  digitalWrite(DIR, HIGH); 
  
  analogWriteFrequency(STEP, STEPPER_FREQ);
  analogWriteResolution(12);

  SerialUSB.begin(9600);


  // stepperTimer.begin()

}

float newAngle = 0;
float currentAngle = 0;
float angleIncrement = 0;
int angleIncrementDir = 1;
unsigned long setpointTime = micros();

void science_loop () {
  //writeToStepperMotor(1800,1);
  // stepper.setMicrostep(1);
  /*
  if(Serial.available()){
    newAngle = Serial.parseFloat();
    Serial.print("New Setpoint: ");
    Serial.println(newAngle);
    currentAngle -= (angleIncrement - (micros() - setpointTime) * TIME_ANGLE_CNST) * angleIncrementDir; 
    setpointTime = micros();
    stepperTimer.end();
  }

  if (newAngle != currentAngle){
    analogWrite(STEP, 2048);
    angleIncrement = abs(newAngle - currentAngle);
    Serial.println(angleIncrement);
    angleIncrementDir = (newAngle > currentAngle) ? 1 : -1;
    digitalWrite(DIR, (angleIncrementDir > 0) ? 1 : 0);
    stepperTimer.begin(stepperISR, angleIncrement * (1/TIME_ANGLE_CNST));
    currentAngle = newAngle;
  }
  */
  // setSpeed(50, MOT1_PWM, MOT1_DIR); 
  // delay(5000); 
  // setSpeed(0, MOT1_PWM, MOT1_DIR);
  // delay(1000); 
  // setSpeed(-50, MOT1_PWM, MOT1_DIR); 
  // delay(5000);

  SerialUSB.println("Forwards");
  digitalWrite(MOT1_DIR, HIGH); 
  analogWrite(MOT1_PWM, 128);
  delay(5000);
  SerialUSB.println("Stopped");
  digitalWrite(MOT1_DIR, HIGH); 
  analogWrite(MOT1_PWM, 128);
  delay(1000);
  SerialUSB.println("Reverse");
  digitalWrite(MOT1_DIR, LOW); 
  analogWrite(MOT1_PWM, 128);
  delay(5000);


  // analogWrite(STEP, 2048);
  // delay(900);
  // analogWrite(STEP, 0);
  // delay(400);

  

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