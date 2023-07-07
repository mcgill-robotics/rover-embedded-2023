#include <Arduino.h>
#include "BLDCMotor.h"

#define LB_HALL_A_PIN 23
#define LB_HALL_B_PIN 22
#define LB_HALL_C_PIN 21
#define LB_PWM_PIN 5
#define LF_HALL_A_PIN 17
#define LF_HALL_B_PIN 16
#define LF_HALL_C_PIN 15
#define LF_PWM_PIN 3
#define RB_HALL_A_PIN 20
#define RB_HALL_B_PIN 19
#define RB_HALL_C_PIN 18
#define RB_PWM_PIN 4
#define RF_HALL_A_PIN 10
#define RF_HALL_B_PIN 11
#define RF_HALL_C_PIN 12
#define RF_PWM_PIN 2

#define SETUP_TIME_DELAY 1000

void drive_setup();
void drive_loop();

void attachAllInterrupts();
void measureSpeeds();
void takeReadings();
void updateControllers();
void writeSpeeds();
void lb_hall_a_int();
void lb_hall_b_int();
void lb_hall_c_int();
void lf_hall_a_int();
void lf_hall_b_int();
void lf_hall_c_int();
void rb_hall_a_int();
void rb_hall_b_int();
void rb_hall_c_int();
void rf_hall_a_int();
void rf_hall_b_int();
void rf_hall_c_int();