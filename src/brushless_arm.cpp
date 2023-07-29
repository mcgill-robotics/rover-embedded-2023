#ifdef BRUSHLESS_ARM // LEAVE THIS AT THE TOP OF THIS FILE
#include "rover_arm.h"
// Drivers.
#include "RoverArmMotor.h"
#include "AMT22.h"
#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>

// Standard includes.
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <bitset>
#include <limits>

float armBrushlessTargetAngles[3];
float armBrushlessActualAngles[3];

static unsigned long lastTime;

#define MIN_FLOAT -std::numeric_limits<float>::infinity()
#define MAX_FLOAT std::numeric_limits<float>::infinity()
static void attach_all_interrupts();

#define DEBOUNCE_DELAY 100

volatile int limit_elbow_max_activated = 0;
volatile int limit_elbow_min_activated = 0;
volatile int limit_shoulder_max_activated = 0;
volatile int limit_shoulder_min_activated = 0;
volatile int limit_waist_max_activated = 0;
volatile int limit_waist_min_activated = 0;

volatile unsigned long last_trigger_time_elbow_max = 0;
volatile unsigned long last_trigger_time_elbow_min = 0;
volatile unsigned long last_trigger_time_shoulder_max = 0;
volatile unsigned long last_trigger_time_shoulder_min = 0;
volatile unsigned long last_trigger_time_waist_max = 0;
volatile unsigned long last_trigger_time_waist_min = 0;

/*---------------------ELBOW_SERVO DECLARATIONS---------------------*/
#if TEST_ELBOW_SERVO == 1
RoverArmMotor Elbow(armBrushlessTargetAngles[0], PWM2, -1, CS1, BLUE_ROBOTICS, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
#endif

/*---------------------SHOULDER_SERVO DECLARATIONS---------------------*/
#if TEST_SHOULDER_SERVO == 1
RoverArmMotor Shoulder(armBrushlessTargetAngles[1], PWM1, -1, CS2, BLUE_ROBOTICS, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE);
#endif

/*---------------------WAIST_SERVO DECLARATIONS---------------------*/
#if TEST_WAIST_SERVO == 1
RoverArmMotor Waist(armBrushlessTargetAngles[2], PWM3, -1, CS3, BLUE_ROBOTICS, WAIST_MIN_ANGLE, WAIST_MAX_ANGLE);
#endif

void brushless_arm_setup()
{
  // Initiate SPI bus.
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64);

  /* ELBOW_SERVO setup */
#if TEST_ELBOW_SERVO == 1
  Elbow.setAngleLimits(ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
  Elbow.stop_tick = 1;
  Elbow.fight_gravity = 1;
  Waist.error_range = 3.0f;
  Elbow.set_safety_pins(ELBOW_BRAKE, LIMIT_ELBOW_MAX, LIMIT_ELBOW_MIN);

  Elbow.begin(REG_KP_ELBOW, REG_KI_ELBOW, REG_KD_ELBOW, REG_KP_ELBOW_AGG, REG_KI_ELBOW_AGG, REG_KD_ELBOW_AGG);

  Elbow.reset_encoder();
  Elbow.set_zero_angle();
  Elbow.set_current_as_zero_angle_sw(ELBOW_ZERO_ANGLE);
  Elbow.new_setpoint(0.0);
#endif

  /* SHOULDER_SERVO setup */
#if TEST_SHOULDER_SERVO == 1
  Shoulder.setAngleLimits(SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE);
  Shoulder.stop_tick = 1;
  Shoulder.error_range = 1.0f;
  Shoulder.set_safety_pins(SHOULDER_BRAKE, LIMIT_SHOULDER_MAX, LIMIT_SHOULDER_MIN);

  Shoulder.begin(REG_KP_SHOULDER, REG_KI_SHOULDER, REG_KD_SHOULDER, REG_KP_SHOULDER_AGG, REG_KI_SHOULDER_AGG, REG_KD_SHOULDER_AGG);

  Shoulder.reset_encoder();
  Shoulder.set_zero_angle();
  Shoulder.set_current_as_zero_angle_sw(SHOULDER_ZERO_ANGLE);
  Shoulder.new_setpoint(0.0);
#endif

  /*---WAIST_SERVO setup---*/
#if TEST_WAIST_SERVO == 1
  Waist.wrist_waist = 0;
  Waist.stop_tick = 1;
  Waist.inverted_angle = 1;
  Waist.error_range = 2.0f;
  Waist.set_gear_ratio(WAIST_GEAR_RATIO);
  Waist.setAngleLimits(WAIST_MIN_ANGLE, WAIST_MAX_ANGLE);
  Waist.set_safety_pins(-1, LIMIT_WAIST_MAX, LIMIT_WAIST_MIN);

  Waist.begin(REG_KP_WAIST, REG_KI_WAIST, REG_KD_WAIST, REG_KP_WAIST_AGG, REG_KI_WAIST_AGG, REG_KD_WAIST_AGG);

  Waist.reset_encoder();
  Waist.set_zero_angle();
  Waist.set_current_as_zero_angle_sw(WAIST_ZERO_ANGLE);
  Waist.new_setpoint(0.0);
#endif

  attach_all_interrupts();
}

void brushless_arm_loop()
{
  while (micros() < lastTime + PID_PERIOD_US)
    ;
  lastTime += PID_PERIOD_US;
// put your main code here, to run repeatedly:
#if TEST_ELBOW_SERVO == 1
  Elbow.tick();
#endif
#if TEST_SHOULDER_SERVO == 1
  Shoulder.tick();
#endif
#if TEST_WAIST_SERVO == 1
  Waist.tick();
#endif
}

#if TEST_ELBOW_SERVO == 1
void limit_elbow_max_int()
{
  unsigned long now = millis();
  if (now - last_trigger_time_elbow_max > DEBOUNCE_DELAY)
  {
    last_trigger_time_elbow_max = now;
    if (digitalRead(LIMIT_ELBOW_MAX) == LOW)
    {
      limit_elbow_max_activated = 1;
      Elbow.new_setpoint(Elbow.setpoint - 5.0f);
      Elbow.stop();
    }
    else
    {
      limit_elbow_max_activated = 0;
    }
  }
}

void limit_elbow_min_int()
{
  unsigned long now = millis();
  if (now - last_trigger_time_elbow_min > DEBOUNCE_DELAY)
  {
    last_trigger_time_elbow_min = now;
    if (digitalRead(LIMIT_ELBOW_MIN) == LOW)
    {
      limit_elbow_min_activated = 1;
      Elbow.new_setpoint(Elbow.setpoint + 5.0f);
      Elbow.stop();
    }
    else
    {
      limit_elbow_min_activated = 0;
    }
  }
}
#endif

#if TEST_SHOULDER_SERVO == 1
void limit_shoulder_max_int()
{
  unsigned long now = millis();
  if (now - last_trigger_time_shoulder_max > DEBOUNCE_DELAY)
  {
    last_trigger_time_shoulder_max = now;
    if (digitalRead(LIMIT_SHOULDER_MAX) == LOW)
    {
      limit_shoulder_max_activated = 1;
      Shoulder.new_setpoint(Shoulder.setpoint - 5.0f);
      Shoulder.stop();
    }
    else
    {
      limit_shoulder_max_activated = 0;
    }
  }
}

void limit_shoulder_min_int()
{
  unsigned long now = millis();
  if (now - last_trigger_time_shoulder_min > DEBOUNCE_DELAY)
  {
    last_trigger_time_shoulder_min = now;
    if (digitalRead(LIMIT_SHOULDER_MIN) == LOW)
    {
      limit_shoulder_min_activated = 1;
      Shoulder.new_setpoint(Shoulder.setpoint + 5.0f);
      Shoulder.stop();
    }
    else
    {
      limit_shoulder_min_activated = 0;
    }
  }
}
#endif

#if TEST_WAIST_SERVO == 1
void limit_waist_max_int()
{
  unsigned long now = millis();
  if (now - last_trigger_time_waist_max > DEBOUNCE_DELAY)
  {
    last_trigger_time_waist_max = now;
    if (digitalRead(LIMIT_WAIST_MAX) == LOW)
    {
      limit_waist_max_activated = 1;
      Waist.new_setpoint(Waist.setpoint - 5.0f);
      Waist.stop();
    }
    else
    {
      limit_waist_max_activated = 0;
    }
  }
}

void limit_waist_min_int()
{
  unsigned long now = millis();
  if (now - last_trigger_time_waist_min > DEBOUNCE_DELAY)
  {
    last_trigger_time_waist_min = now;
    if (digitalRead(LIMIT_WAIST_MIN) == LOW)
    {
      limit_waist_min_activated = 1;
      Waist.new_setpoint(Waist.setpoint + 5.0f);
      Waist.stop();
    }
    else
    {
      limit_waist_min_activated = 0;
    }
  }
}
#endif

void attach_all_interrupts()
{
#if TEST_ELBOW_SERVO == 1
  attachInterrupt(digitalPinToInterrupt(LIMIT_ELBOW_MAX), limit_elbow_max_int, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIMIT_ELBOW_MIN), limit_elbow_min_int, CHANGE);
#endif

#if TEST_SHOULDER_SERVO == 1
  attachInterrupt(digitalPinToInterrupt(LIMIT_SHOULDER_MAX), limit_shoulder_max_int, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SHOULDER_MIN), limit_shoulder_min_int, CHANGE);
#endif

#if TEST_WAIST_SERVO == 1
  attachInterrupt(digitalPinToInterrupt(LIMIT_WAIST_MAX), limit_waist_max_int, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIMIT_WAIST_MIN), limit_waist_min_int, CHANGE);
#endif
}

#endif // LEAVE THIS AT THE BOTTOM OF THIS FILE