#include <Arduino.h>

#define LB_HALL_A_PIN PC8
#define LB_HALL_B_PIN PC9// PC_9
#define LB_HALL_C_PIN PC7 // PC_7
#define LB_PWM_PIN PB3 // PB3
// #define LF_HALL_A_PIN PC_9
// #define LF_HALL_B_PIN PC_9
// #define LF_HALL_C_PIN PC_9
// #define LF_PWM_PIN PC_9
// #define RB_HALL_A_PIN PC_9
// #define RB_HALL_B_PIN PC_9
// #define RB_HALL_C_PIN PC_9
// #define RB_PWM_PIN PC_9
// #define RF_HALL_A_PIN PC_9
// #define RF_HALL_B_PIN PC_9
// #define RF_HALL_C_PIN PC_9
// #define RF_PWM_PIN PC_9

#define PID_KP 1
#define PID_KD 1
#define PID_KI 1
#define SETUP_TIME_DELAY 3000

static volatile unsigned long lastTime = micros(); //Time of MCU when last read hall sensors

void drive_setup();
void drive_loop();

//Interrupt functions
void attachAllInterrupts();
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
