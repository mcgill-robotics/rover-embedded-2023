#ifdef DRIVE //LEAVE THIS AT THE TOP OF THIS FILE

#include <Arduino.h>
#include "drive_main.h"
#include "WheelMotor.h"
#include "KickFFT.h"
#include "TimerInterrupt_Generic.h"
#include "PID_v1.h"

// Something inside here is clobbering something
WheelMotor LBMotor(LB_PWM_PIN, LB_HALL_A_PIN, LB_HALL_B_PIN, LB_HALL_C_PIN);
// WheelMotor LFMotor(LF_PWM_PIN, LF_HALL_A_PIN, LF_HALL_B_PIN, LF_HALL_C_PIN);
// WheelMotor RBMotor(RB_PWM_PIN, RB_HALL_A_PIN, RB_HALL_B_PIN, RB_HALL_C_PIN);
// WheelMotor RFMotor(RF_PWM_PIN, RF_HALL_A_PIN, RF_HALL_B_PIN, RF_HALL_C_PIN);
double real_speed, motor_us, target_speed;

// PID LBPid(&real_speed, &motor_us, &target_speed, PID_KP, PID_KI, PID_KD, 1);
// PID LFPid(&LFMotor.real_speed, &LFMotor.motor_us, &LFMotor.target_speed, PID_KP, PID_KI, PID_KD, 1);
// PID RBPid(&RBMotor.real_speed, &RBMotor.motor_us, &RBMotor.target_speed, PID_KP, PID_KI, PID_KD, 1);
// PID RFPid(&RFMotor.real_speed, &RFMotor.motor_us, &RFMotor.target_speed, PID_KP, PID_KI, PID_KD, 1);

const uint16_t samples = 512;
int16_t input[samples];
volatile uint16_t input_buffer[samples];
volatile int sample_count;
const float Fs = 32000;
uint32_t mag[samples] = {0};
uint32_t phase[samples] = {0};
uint16_t startIndex = 0;
uint16_t endIndex = 0;
volatile int32_t max_mag = 0;
volatile int32_t max_freq, max_phase = 0;
volatile uint16_t halla, hallb, hallc;
int speed;
uint16_t freqs_cpy[samples], phases_cpy[samples], mags_cpy[samples];
enum directionStates{AWAITING_AB, AWAITING_SECOND_SIGNAL, FORWARD_PLAUSIBLE, BACKWARD_PLAUSIBLE, FORWARD_DECIDED, BACKWARD_DECIDED};
enum directions{FORWARDS=1, BACKWARDS=-1};
volatile int currentDirection = FORWARDS;
volatile int currentState = AWAITING_AB;
volatile int forward_estimations, backward_estimations;


STM32Timer ITimer(TIM1);
STM32Timer TimerSquared(TIM2);

volatile float test_lb_halla, test_lb_hallb, test_lb_hallc;
volatile unsigned long time_since;
float percentage_sent;

volatile int16_t reading;

void takeReading();
void integrateReadings();

void printShit();

void drive_setup() {
  //CANbus setup?

  //attaches all interrupts for the hall sensors
  Serial.println("Setting up Interrupts");
  // attachAllInterrupts();
  Serial.println("Interrupts done");
  delay(2000);

  Serial.begin(9600);
  target_speed = 1800;
  // LBMotor.writeSpeed();
  Serial.println("Setup done");
  delay(SETUP_TIME_DELAY);
  delay(2000);
    // LBMotor.motor_us = 1750;
  // LBMotor.writeSpeed();
  int sampling_period = (int) (1000000 / Fs); 
  ITimer.attachInterruptInterval(sampling_period, takeReading);
  TimerSquared.attachInterruptInterval(sampling_period * samples, integrateReadings);

}

void drive_loop() {
  //Measure speed by getting average over MOVING_AVG_SIZE values
  // LBMotor.measureSpeed();
  // LFMotor.measureSpeed();
  // RBMotor.measureSpeed();
  // RFMotor.measureSpeed();

  // speed = map(max_freq, -897, 897, -100, 100);
  // real_speed = speed;

  //Compute the PID calculations to get an updated value to send to the motors
  // LBPid.Compute();
  // LFPid.Compute();
  // RBPid.Compute();
  // RFPid.Compute();
  // LBMotor.writeSpeed();

  for(int i=1500; i<=1900; i++){
    // LBMotor.motor_us = i;
    percentage_sent = (float) (i-1500)/4.0f;
    LBMotor.motor_us = i;
    LBMotor.writeSpeed();
    speed = map(max_freq, -897, 897, -100, 100);
    if(abs(speed) > 150) speed = 0;
    // LBMotor.real_speed = speed;
    printShit();
    delay(10);
  }
// Serial.println("Going Down");
delay(2000);
  for(int i=1900; i>=1100; i--){
    LBMotor.motor_us = i;
    percentage_sent = (float) (i-1500)/4.0f;
    LBMotor.writeSpeed();
    speed = map(max_freq, -897, 897, -100, 100);
    if(abs(speed) > 150) speed = 0;
    // LBMotor.real_speed = speed;
    printShit();
    delay(10);
  }
//Serial.println("Going Down");
delay(2000);
  for(int i=1100; i<=1500; i++){
    LBMotor.motor_us = i;
    percentage_sent = (float) (i-1500)/4.0f;
    LBMotor.writeSpeed();
    speed = map(max_freq, -897, 897, -100, 100);
    if(abs(speed) > 150) speed = 0;
    // LBMotor.real_speed = speed;
    printShit();
    delay(10);
  }

}

void printShit(){
    Serial.print(percentage_sent);
    Serial.print(",");
    // Serial.print(max_mag);
    // Serial.print(",");
    // Serial.print(max_freq);
    // Serial.print(",");
    Serial.println(speed);
    // for(uint16_t j = 0; j < 512; j++)
    // {
    //   Serial.print(mag[j]);
    //   Serial.print(",");
    //   Serial.println(phase[j]);
    // }
}

void integrateReadings(){
    if(sample_count >= samples)
    {
    ITimer.disableTimer();
    // ITimer.detachInterrupt();

    memcpy(input, (int16_t*)input_buffer, 2 * samples);
    // input = input_buffer;
    KickFFT<int16_t>::fft(Fs, 0, Fs/2, samples, input, mag, phase, startIndex, endIndex);

    // Serial.println("Freq(Hz),Magnitude");
  
    max_mag = 0;
    max_freq = 0;
    max_phase = 0;
    for(uint16_t i = startIndex; i < endIndex; i++)
    {
      if(mag[i] >= max_mag && (i != 0))
      {
        max_mag = mag[i];
        max_phase = phase[i];
        max_freq = (i*Fs/samples);
      }

    }
    bool forwards = true;

    if(backward_estimations <= forward_estimations){
      if(currentDirection == FORWARDS && abs(max_freq) < 300){
        currentDirection = BACKWARDS;
      }
    }else if(forward_estimations <= backward_estimations){
      if(currentDirection == BACKWARDS && abs(max_freq) < 300){
        currentDirection = FORWARDS;
      }
    }
    max_freq *= (currentDirection);

    if ((backward_estimations - forward_estimations <= 0) && max_freq < 300){
      forwards = false;
    }
    else if(forward_estimations - backward_estimations <= 0 && max_freq > 300){

    }
    
    forward_estimations = 0;
    backward_estimations = 0;

    sample_count = 0;
    // ITimer.attachInterruptInterval(1000000/Fs, takeReading);
    ITimer.enableTimer();
    }
  }

void takeReading(){
  if(sample_count < samples){
    int A = digitalRead(LB_HALL_A_PIN);
    int B = digitalRead(LB_HALL_B_PIN);
    int C = digitalRead(LB_HALL_C_PIN);

    input_buffer[sample_count] = A;
    sample_count++;

    int AB = (A & B) & (~C);
    int AC = (A & C) & (~B);
    int BC = (C & B) & (~A);

    if(AB)
    {

      switch(currentState)
      {
        case AWAITING_AB:
          currentState = AWAITING_SECOND_SIGNAL;
          break;
        case FORWARD_DECIDED:
          forward_estimations++;
          currentState = AWAITING_AB;
          break;
        case BACKWARD_DECIDED:
          backward_estimations++;
          currentState = AWAITING_AB;
          break;
      }

    }else if(BC)
    {
      switch(currentState)
      {
        case AWAITING_SECOND_SIGNAL:
          currentState = FORWARD_PLAUSIBLE;
          break;
        case BACKWARD_PLAUSIBLE:
          currentState = BACKWARD_DECIDED;
          break;
      }
    }else if(AC)
    {
      switch(currentState)
      {
        case AWAITING_SECOND_SIGNAL:
          currentState = BACKWARD_PLAUSIBLE;
          break;
        case FORWARD_PLAUSIBLE:
          currentState = FORWARD_DECIDED;
          break;
      }
    }
  }
}

#endif