#ifdef DRIVE //LEAVE THIS AT THE TOP OF THIS FILE

#include <Arduino.h>
#include "drive_main.h"
#include "WheelMotor.h"

WheelMotor LBMotor(LB_PWM_PIN, LB_HALL_A_PIN, LB_HALL_B_PIN, LB_HALL_C_PIN);
WheelMotor LFMotor(LF_PWM_PIN, LF_HALL_A_PIN, LF_HALL_B_PIN, LF_HALL_C_PIN);
WheelMotor RBMotor(RB_PWM_PIN, RB_HALL_A_PIN, RB_HALL_B_PIN, RB_HALL_C_PIN);
WheelMotor RFMotor(RF_PWM_PIN, RF_HALL_A_PIN, RF_HALL_B_PIN, RF_HALL_C_PIN);

void drive_setup() {
  attachAllInterrupts();
}

void drive_loop() {
  // put your main code here, to run repeatedly:
}

void attachAllInterrupts(){
  attachInterrupt(LB_HALL_A_PIN, lb_hall_a_int, RISING);
  attachInterrupt(LB_HALL_A_PIN, lb_hall_a_int, RISING);
  attachInterrupt(LB_HALL_A_PIN, lb_hall_a_int, RISING);

  attachInterrupt(LF_HALL_A_PIN, lf_hall_a_int, RISING);
  attachInterrupt(LF_HALL_A_PIN, lf_hall_a_int, RISING);
  attachInterrupt(LF_HALL_A_PIN, lf_hall_a_int, RISING);

  attachInterrupt(RB_HALL_A_PIN, rb_hall_a_int, RISING);
  attachInterrupt(RB_HALL_A_PIN, rb_hall_a_int, RISING);
  attachInterrupt(RB_HALL_A_PIN, rb_hall_a_int, RISING);

  attachInterrupt(RF_HALL_A_PIN, rf_hall_a_int, RISING);
  attachInterrupt(RF_HALL_A_PIN, rf_hall_a_int, RISING);
  attachInterrupt(RF_HALL_A_PIN, rf_hall_a_int, RISING);
}

void lb_hall_a_int(){
  LBMotor.hall_a_interrupts++;
}

void lb_hall_b_int(){
  LBMotor.hall_b_interrupts++;
}

void lb_hall_c_int(){
  LBMotor.hall_c_interrupts++;
}

void lf_hall_a_int(){
  LFMotor.hall_a_interrupts++;
}

void lf_hall_b_int(){
  LFMotor.hall_b_interrupts++;
}

void lf_hall_c_int(){
  LFMotor.hall_c_interrupts++;
}

void rb_hall_a_int(){
  RBMotor.hall_a_interrupts++;
}

void rb_hall_b_int(){
  RBMotor.hall_b_interrupts++;
}

void rb_hall_c_int(){
  RBMotor.hall_c_interrupts++;
}

void rf_hall_a_int(){
  RFMotor.hall_a_interrupts++;
}

void rf_hall_b_int(){
  RFMotor.hall_b_interrupts++;
}

void rf_hall_c_int(){
  RFMotor.hall_c_interrupts++;
}



#endif //LEAVE THIS AT THE BOTTOM OF THIS FILE