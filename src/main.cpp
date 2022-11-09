#include <Arduino.h>
#include "science_main.h"
#include "drive_main.h"
#include "sensor_main.h"
#include "upper_arm_main.h"
#include "lower_arm_main.h"
#include "power_main.h"

#define loop_hertz 100
volatile byte tflag;
void loop_timer_int();

void setup() {
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *loop_timer = new HardwareTimer(Instance);

  loop_timer->setOverflow(loop_hertz, HERTZ_FORMAT);
  loop_timer->attachInterrupt(loop_timer_int);
  loop_timer->resume();
  #ifdef SCIENCE
  science_setup();
  #endif
  #ifdef POWER
  power_setup();
  #endif
  #ifdef SENSOR
  sensor_setup();
  #endif
  #ifdef UPPER_ARM
  upper_arm_setup();
  #endif
  #ifdef LOWER_ARM
  lower_arm_setup();
  #endif
  #ifdef DRIVE
  drive_setup();
  #endif
}

void loop() {
  if(tflag == 1){
    #ifdef SCIENCE
    science_loop();
    #endif
    #ifdef POWER
    power_loop();
    #endif
    #ifdef SENSOR
    sensor_loop();
    #endif
    #ifdef UPPER_ARM
    upper_arm_loop();
    #endif
    #ifdef LOWER_ARM
    lower_arm_loop();
    #endif
    #ifdef DRIVE
    drive_loop();
    #endif
    tflag = 0;
  }
}

void loop_timer_int(){
  tflag = 1;
}