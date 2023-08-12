#ifdef BRUSHED_ARM // LEAVE THIS AT THE TOP OF THIS FILE
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

float armBrushedTargetAngles[3];
float armBrushedActualAngles[3];

static unsigned long lastTime;

#define MIN_FLOAT -std::numeric_limits<float>::infinity()
#define MAX_FLOAT std::numeric_limits<float>::infinity()
static void attach_all_interrupts();

#define DEBOUNCE_DELAY 100

volatile unsigned long last_trigger_time_end_effector_max = 0;
volatile unsigned long last_trigger_time_end_effector_min = 0;
volatile unsigned long last_trigger_time_wrist_pitch_max = 0;
volatile unsigned long last_trigger_time_wrist_pitch_min = 0;

volatile int limit_wrist_pitch_max_activated = 0;
volatile int limit_wrist_pitch_min_activated = 0;
volatile int limit_end_effector_max_activated = 0;
volatile int limit_end_effector_min_activated = 0;

/*---------------------WRIST_ROLL_CYTRON---------------------*/
#if TEST_WRIST_ROLL_CYTRON == 1
RoverArmMotor Wrist_Roll(&armBrushedTargetAngles[1], &armBrushedActualAngles[1], PWM1, DIR1, CS1, CYTRON, WRIST_ROLL_MIN_ANGLE, WRIST_ROLL_MAX_ANGLE);
#endif

/*---------------------WRIST_PITCH_CYTRON---------------------*/
#if TEST_WRIST_PITCH_CYTRON == 1
RoverArmMotor Wrist_Pitch(&armBrushedTargetAngles[2], &armBrushedActualAngles[2], PWM2, DIR2, CS2, CYTRON, WRIST_PITCH_MIN_ANGLE, WRIST_PITCH_MAX_ANGLE);
#endif

/*---------------------END_EFFECTOR_CYTRON---------------------*/
#if TEST_END_EFFECTOR_CYTRON == 1
RoverArmMotor End_Effector(&armBrushedTargetAngles[0], &armBrushedActualAngles[0], PWM3, DIR3, CS3, CYTRON, MIN_FLOAT, MAX_FLOAT);
#endif

void brushed_arm_setup()
{
  // Initiate SPI bus.
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64);

  /*---WRIST_ROLL_CYTRON setup---*/
#if TEST_WRIST_ROLL_CYTRON == 1
  // Wrist_Roll.wrist_waist = 1;
  // Wrist_Roll.stop_tick = 1;
  Wrist_Roll.set_gear_ratio(WRIST_ROLL_GEAR_RATIO);
  Wrist_Roll.setAngleLimits(WRIST_ROLL_MIN_ANGLE, WRIST_ROLL_MAX_ANGLE);

  Wrist_Roll.begin(REG_KP_WRIST_ROLL, REG_KI_WRIST_ROLL, REG_KD_WRIST_ROLL,
                   REG_KP_WRIST_ROLL_AGG, REG_KI_WRIST_ROLL_AGG, REG_KD_WRIST_ROLL_AGG);

  // Assume at zero angle at startup.
  Wrist_Roll.set_current_as_zero_angle_sw();
  Wrist_Roll.new_setpoint(0.0);
#endif

  /*---WRIST_PITCH_CYTRON setup---*/
#if TEST_WRIST_PITCH_CYTRON == 1
  Wrist_Pitch.wrist_waist = 0;
  // Wrist_Pitch.stop_tick = 1;
  Wrist_Pitch.set_gear_ratio(WRIST_PITCH_GEAR_RATIO);
  Wrist_Pitch.setAngleLimits(WRIST_PITCH_MIN_ANGLE, WRIST_PITCH_MAX_ANGLE);
  Wrist_Pitch.set_safety_pins(-1, LIMIT_WRIST_PITCH_MAX, LIMIT_WRIST_PITCH_MIN);

  Wrist_Pitch.begin(REG_KP_WRIST_PITCH, REG_KI_WRIST_PITCH, REG_KD_WRIST_PITCH,
                    REG_KP_WRIST_PITCH_AGG, REG_KI_WRIST_PITCH_AGG, REG_KD_WRIST_PITCH_AGG);

  Wrist_Pitch.reset_encoder();
  Wrist_Pitch.set_zero_angle();
  Wrist_Pitch.set_current_as_zero_angle_sw(WRIST_PITCH_ZERO_ANGLE);
  Wrist_Pitch.new_setpoint(0.0);
#endif

  /*---END_EFFECTOR_CYTRON setup---*/
#if TEST_END_EFFECTOR_CYTRON == 1
  End_Effector.wrist_waist = 0;
  End_Effector.setAngleLimits(MIN_FLOAT, MAX_FLOAT);
  End_Effector.set_safety_pins(-1, LIMIT_END_EFFECTOR_MAX, LIMIT_END_EFFECTOR_MIN);
  pinMode(END_EFFECTOR_LASER, OUTPUT);
  digitalWrite(END_EFFECTOR_LASER, HIGH);
  End_Effector.begin(0, 0, 0, 0, 0, 0);
#endif

  attach_all_interrupts();
}

void brushed_arm_loop()
{
  while (micros() < lastTime + PID_PERIOD_US)
    ;
  lastTime += PID_PERIOD_US;
#if TEST_WRIST_ROLL_CYTRON == 1
  Wrist_Roll.tick();
#endif
#if TEST_WRIST_PITCH_CYTRON == 1
  Wrist_Pitch.tick();
#endif
#if TEST_END_EFFECTOR_CYTRON == 1
  if (*(End_Effector.setpoint) < 0)
  {
    digitalWrite(End_Effector._dir, LOW);
  }
  else
  {
    digitalWrite(End_Effector._dir, HIGH);
  }
  (End_Effector.pwmInstance)->setPWM(End_Effector._pwm, End_Effector._pwm_freq, abs(*(End_Effector.setpoint)));
#endif
}

#if TEST_WRIST_PITCH_CYTRON == 1
void limit_wrist_pitch_max_int()
{
  unsigned long now = millis();
  if (now - last_trigger_time_wrist_pitch_max > DEBOUNCE_DELAY)
  {
    last_trigger_time_wrist_pitch_max = now;
    if (digitalRead(LIMIT_WRIST_PITCH_MAX) == LOW)
    {
      limit_wrist_pitch_max_activated = 1;
      Wrist_Pitch.stop();
      // Wrist_Pitch.set_current_as_angle_sw(Wrist_Pitch.max_angle);
      Wrist_Pitch.new_setpoint(*(Wrist_Pitch.setpoint) - 5.0f);
    }
    else
    {
      limit_wrist_pitch_max_activated = 0;
    }
  }
}

void limit_wrist_pitch_min_int()
{
  unsigned long now = millis();
  if (now - last_trigger_time_wrist_pitch_min > DEBOUNCE_DELAY)
  {
    last_trigger_time_wrist_pitch_min = now;
    bool is_low = digitalRead(LIMIT_WRIST_PITCH_MIN) == LOW;
    limit_wrist_pitch_min_activated = is_low;
    if (is_low)
    {
      limit_wrist_pitch_min_activated = 1;
      Wrist_Pitch.stop();
      // Wrist_Pitch.set_current_as_angle_sw(Wrist_Pitch.min_angle);
      Wrist_Pitch.new_setpoint(*(Wrist_Pitch.setpoint) + 5.0f);
    }
    else
    {
      limit_wrist_pitch_min_activated = 0;
    }
  }
}
#endif

#if TEST_END_EFFECTOR_CYTRON == 1
void limit_end_effector_max_int()
{
  unsigned long now = millis();
  if (now - last_trigger_time_end_effector_max > DEBOUNCE_DELAY)
  {
    last_trigger_time_end_effector_max = now;
    if (digitalRead(LIMIT_END_EFFECTOR_MAX) == LOW)
    {
      limit_end_effector_max_activated = 1;
      End_Effector.stop();
      End_Effector.reverse();
      End_Effector.new_setpoint(-10.0f);
    }
    else
    {
      limit_end_effector_max_activated = 0;
      End_Effector.stop();
    }
  }
}

void limit_end_effector_min_int()
{
  unsigned long now = millis();
  if (now - last_trigger_time_end_effector_min > DEBOUNCE_DELAY)
  {
    last_trigger_time_end_effector_min = now;
    if (digitalRead(LIMIT_END_EFFECTOR_MIN) == LOW)
    {
      limit_end_effector_min_activated = 1;
      End_Effector.stop();
      End_Effector.forward();
      End_Effector.new_setpoint(10.0f);
    }
    else
    {
      limit_end_effector_min_activated = 0;
      End_Effector.stop();
    }
  }
}
#endif

void attach_all_interrupts()
{
#if TEST_WRIST_PITCH_CYTRON == 1
  attachInterrupt(digitalPinToInterrupt(LIMIT_WRIST_PITCH_MAX), limit_wrist_pitch_max_int, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIMIT_WRIST_PITCH_MIN), limit_wrist_pitch_min_int, CHANGE);
#endif

#if TEST_END_EFFECTOR_CYTRON == 1
  attachInterrupt(digitalPinToInterrupt(LIMIT_END_EFFECTOR_MAX), limit_end_effector_max_int, FALLING_EDGE);
  attachInterrupt(digitalPinToInterrupt(LIMIT_END_EFFECTOR_MIN), limit_end_effector_min_int, FALLING_EDGE);
#endif
}
#endif // LEAVE THIS AT THE BOTTOM OF THIS FILE