#include <Arduino.h>
#include "science_main.h"
#include "drive_main.h"
#include "sensor_main.h"
#include "upper_arm_main.h"
#include "lower_arm_main.h"
#include "power_main.h"
#include "Can.h"

#define loop_hertz 100
void canbus_timer_int();

void setup() {
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *canbus_timer = new HardwareTimer(Instance);

  canbus_timer->setOverflow(loop_hertz, HERTZ_FORMAT);
  canbus_timer->attachInterrupt(canbus_timer_int);

  CANBus::Message write_buffer[25];
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
  canbus_timer->resume();
}

void loop() {
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
}

void loop_timer_int(){
  
}