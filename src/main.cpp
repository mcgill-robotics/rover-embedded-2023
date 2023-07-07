#include <Arduino.h>
#include "science.h"
#include "drive.h"
#include "killswitch.h"
#include "brushed_arm.h"
#include "brushless_arm.h"
#include "power.h"

#define CONTROL_LOOP_PERIOD_US 10000

unsigned long lastTime, microsLeft;

void setup() {
  #ifdef SCIENCE
  science_setup();
  #endif
  #ifdef POWER_SYS
  power_setup();
  #endif
  #ifdef KILLSWITCH
  killswitch_setup();
  #endif
  #ifdef BRUSHED_ARM
  brushed_arm_setup();
  #endif
  #ifdef BRUSHLESS_ARM
  brushless_arm_setup();
  #endif
  #ifdef DRIVE
  drive_setup();
  #endif
  lastTime = micros();
}

void loop() {
  microsLeft = micros() - lastTime;
  delayMicroseconds(microsLeft);
  while(micros() < lastTime + CONTROL_LOOP_PERIOD_US);
  lastTime += CONTROL_LOOP_PERIOD_US;
  #ifdef SCIENCE
  science_loop();
  #endif
  #ifdef POWER_SYS
  power_loop();
  #endif
  #ifdef KILLSWITCH
  killswitch_loop();
  #endif
  #ifdef BRUSHED_ARM
  brushed_arm_loop();
  #endif
  #ifdef BRUSHLESS_ARM
  brushless_arm_loop();
  #endif
  #ifdef DRIVE
  drive_loop();
  #endif
}