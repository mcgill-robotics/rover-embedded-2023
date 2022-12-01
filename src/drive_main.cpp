#ifdef DRIVE //LEAVE THIS AT THE TOP OF THIS FILE

#include <Arduino.h>
#include "drive_main.h"
#include "WheelMotor.h"
#include "PID_v1.h"

WheelMotor LBMotor(LB_PWM_PIN, LB_HALL_A_PIN, LB_HALL_B_PIN, LB_HALL_C_PIN);
WheelMotor LFMotor(LF_PWM_PIN, LF_HALL_A_PIN, LF_HALL_B_PIN, LF_HALL_C_PIN);
WheelMotor RBMotor(RB_PWM_PIN, RB_HALL_A_PIN, RB_HALL_B_PIN, RB_HALL_C_PIN);
WheelMotor RFMotor(RF_PWM_PIN, RF_HALL_A_PIN, RF_HALL_B_PIN, RF_HALL_C_PIN);

PID LBPid(&LBMotor.real_speed, &LBMotor.motor_us, &LBMotor.target_speed, PID_KP, PID_KI, PID_KD, 1);
PID LFPid(&LFMotor.real_speed, &LFMotor.motor_us, &LFMotor.target_speed, PID_KP, PID_KI, PID_KD, 1);
PID RBPid(&RBMotor.real_speed, &RBMotor.motor_us, &RBMotor.target_speed, PID_KP, PID_KI, PID_KD, 1);
PID RFPid(&RFMotor.real_speed, &RFMotor.motor_us, &RFMotor.target_speed, PID_KP, PID_KI, PID_KD, 1);

void drive_setup() {
  //CANbus setup?

  //attaches all interrupts for the hall sensors
  attachAllInterrupts();
  
  delay(SETUP_TIME_DELAY);
}

void drive_loop() {
  //Measure speed by getting average over MOVING_AVG_SIZE values
  LBMotor.measureSpeed();
  LFMotor.measureSpeed();
  RBMotor.measureSpeed();
  RFMotor.measureSpeed();

  //Compute the PID calculations to get an updated value to send to the motors
  LBPid.Compute();
  LFPid.Compute();
  RBPid.Compute();
  RFPid.Compute();

  //Send the calculated speeds to the motors
  LBMotor.writeSpeed();
  LFMotor.writeSpeed();
  RBMotor.writeSpeed();
  RFMotor.writeSpeed();
}

/// @brief Setup function that attaches all interrupts for the hall sensors
void attachAllInterrupts(){
  attachInterrupt(LB_HALL_A_PIN, lb_hall_a_int, RISING);
  attachInterrupt(LB_HALL_B_PIN, lb_hall_b_int, RISING);
  attachInterrupt(LB_HALL_C_PIN, lb_hall_c_int, RISING);

  attachInterrupt(LF_HALL_A_PIN, lf_hall_a_int, RISING);
  attachInterrupt(LF_HALL_B_PIN, lf_hall_b_int, RISING);
  attachInterrupt(LF_HALL_C_PIN, lf_hall_c_int, RISING);

  attachInterrupt(RB_HALL_A_PIN, rb_hall_a_int, RISING);
  attachInterrupt(RB_HALL_B_PIN, rb_hall_b_int, RISING);
  attachInterrupt(RB_HALL_C_PIN, rb_hall_c_int, RISING);

  attachInterrupt(RF_HALL_A_PIN, rf_hall_a_int, RISING);
  attachInterrupt(RF_HALL_B_PIN, rf_hall_b_int, RISING);
  attachInterrupt(RF_HALL_C_PIN, rf_hall_c_int, RISING);
}

//-----------------------------------------LEFT BACK MOTOR ISRS-----------------------------------------------//

void lb_hall_a_int(){
  if(LBMotor.lastHallTriggered == 2){
    if(LBMotor.direction_counter > BAD_HALL_COUNTER_REV) LBMotor.direction_counter--;
  }
  else{
    if(LBMotor.direction_counter < BAD_HALL_COUNTER) LBMotor.direction_counter++;
  }
  LBMotor.direction = (LBMotor.direction_counter >= 0) ? 1 : -1;
  LBMotor.lastHallTriggered = 1;

  float new_value = (5714285.714f * LBMotor.direction) / (micros() - lastTime); //constant is degrees/interrupt *1000000us (360deg*(gearRatio/3)*1000000us) gear ratio = 1/21
  lastTime = micros();
  LBMotor.average_readings[LBMotor.int_pos] = new_value;
  if (LBMotor.int_pos++ > MOVING_AVG_SIZE) LBMotor.int_pos = 0;
}

void lb_hall_b_int(){
  if(LBMotor.lastHallTriggered == 3){
    if(LBMotor.direction_counter > BAD_HALL_COUNTER_REV) LBMotor.direction_counter--;
  }
  else{
    if(LBMotor.direction_counter < BAD_HALL_COUNTER) LBMotor.direction_counter++;
  }
  LBMotor.direction = (LBMotor.direction_counter >= 0) ? 1 : -1;
  LBMotor.lastHallTriggered = 2;

  float new_value = (5714285.714f * LBMotor.direction) / (micros() - lastTime); //constant is degrees/interrupt *1000000us (360deg*(gearRatio/3)*1000000us) gear ratio = 1/21
  lastTime = micros();
  LBMotor.average_readings[LBMotor.int_pos] = new_value;
  if (LBMotor.int_pos++ > MOVING_AVG_SIZE) LBMotor.int_pos = 0;
}

void lb_hall_c_int(){
  if(LBMotor.lastHallTriggered == 1){
    if(LBMotor.direction_counter > BAD_HALL_COUNTER_REV) LBMotor.direction_counter--;
  }
  else{
    if(LBMotor.direction_counter < BAD_HALL_COUNTER) LBMotor.direction_counter++;
  }
  LBMotor.direction = (LBMotor.direction_counter >= 0) ? 1 : -1;
  LBMotor.lastHallTriggered = 3;

  float new_value = (5714285.714f * LBMotor.direction) / (micros() - lastTime); //constant is degrees/interrupt *1000000us (360deg*(gearRatio/3)*1000000us) gear ratio = 1/21
  lastTime = micros();
  LBMotor.average_readings[LBMotor.int_pos] = new_value;
  if (LBMotor.int_pos++ > MOVING_AVG_SIZE) LBMotor.int_pos = 0;
}

//-----------------------------------------LEFT FRONT MOTOR ISRS-----------------------------------------------//

void lf_hall_a_int(){
  if(LFMotor.lastHallTriggered == 2){
    if(LFMotor.direction_counter > BAD_HALL_COUNTER_REV) LFMotor.direction_counter--;
  }
  else{
    if(LFMotor.direction_counter < BAD_HALL_COUNTER) LFMotor.direction_counter++;
  }
  LFMotor.direction = (LFMotor.direction_counter >= 0) ? 1 : -1;
  LFMotor.lastHallTriggered = 1;

  float new_value = (5714285.714f * LFMotor.direction) / (micros() - lastTime); //constant is degrees/interrupt *1000000us (360deg*(gearRatio/3)*1000000us) gear ratio = 1/21
  lastTime = micros();
  LFMotor.average_readings[LFMotor.int_pos] = new_value;
  if (LFMotor.int_pos++ > MOVING_AVG_SIZE) LFMotor.int_pos = 0;
}

void lf_hall_b_int(){
  if(LFMotor.lastHallTriggered == 3){
    if(LFMotor.direction_counter > BAD_HALL_COUNTER_REV) LFMotor.direction_counter--;
  }
  else{
    if(LFMotor.direction_counter < BAD_HALL_COUNTER) LFMotor.direction_counter++;
  }
  LFMotor.direction = (LFMotor.direction_counter >= 0) ? 1 : -1;
  LFMotor.lastHallTriggered = 2;

  float new_value = (5714285.714f * LFMotor.direction) / (micros() - lastTime); //constant is degrees/interrupt *1000000us (360deg*(gearRatio/3)*1000000us) gear ratio = 1/21
  lastTime = micros();
  LFMotor.average_readings[LFMotor.int_pos] = new_value;
  if (LFMotor.int_pos++ > MOVING_AVG_SIZE) LFMotor.int_pos = 0;
}

void lf_hall_c_int(){
  if(LFMotor.lastHallTriggered == 1){
    if(LFMotor.direction_counter > BAD_HALL_COUNTER_REV) LFMotor.direction_counter--;
  }
  else{
    if(LFMotor.direction_counter < BAD_HALL_COUNTER) LFMotor.direction_counter++;
  }
  LFMotor.direction = (LFMotor.direction_counter >= 0) ? 1 : -1;
  LFMotor.lastHallTriggered = 3;

  float new_value = (5714285.714f * LFMotor.direction) / (micros() - lastTime); //constant is degrees/interrupt *1000000us (360deg*(gearRatio/3)*1000000us) gear ratio = 1/21
  lastTime = micros();
  LFMotor.average_readings[LFMotor.int_pos] = new_value;
  if (LFMotor.int_pos++ > MOVING_AVG_SIZE) LFMotor.int_pos = 0;
}

//-----------------------------------------RIGHT BACK MOTOR ISRS-----------------------------------------------//

void rb_hall_a_int(){
  if(RBMotor.lastHallTriggered == 2){
    if(RBMotor.direction_counter > BAD_HALL_COUNTER_REV) RBMotor.direction_counter--;
  }
  else{
    if(RBMotor.direction_counter < BAD_HALL_COUNTER) RBMotor.direction_counter++;
  }
  RBMotor.direction = (RBMotor.direction_counter >= 0) ? 1 : -1;
  RBMotor.lastHallTriggered = 1;

  float new_value = (5714285.714f * RBMotor.direction) / (micros() - lastTime); //constant is degrees/interrupt *1000000us (360deg*(gearRatio/3)*1000000us) gear ratio = 1/21
  lastTime = micros();
  RBMotor.average_readings[RBMotor.int_pos] = new_value;
  if (RBMotor.int_pos++ > MOVING_AVG_SIZE) RBMotor.int_pos = 0;
}

void rb_hall_b_int(){
  if(RBMotor.lastHallTriggered == 3){
    if(RBMotor.direction_counter > BAD_HALL_COUNTER_REV) RBMotor.direction_counter--;
  }
  else{
    if(RBMotor.direction_counter < BAD_HALL_COUNTER) RBMotor.direction_counter++;
  }
  RBMotor.direction = (RBMotor.direction_counter >= 0) ? 1 : -1;
  RBMotor.lastHallTriggered = 2;

  float new_value = (5714285.714f * RBMotor.direction) / (micros() - lastTime); //constant is degrees/interrupt *1000000us (360deg*(gearRatio/3)*1000000us) gear ratio = 1/21
  lastTime = micros();
  RBMotor.average_readings[RBMotor.int_pos] = new_value;
  if (RBMotor.int_pos++ > MOVING_AVG_SIZE) RBMotor.int_pos = 0;
}

void rb_hall_c_int(){
  if(RBMotor.lastHallTriggered == 1){
    if(RBMotor.direction_counter > BAD_HALL_COUNTER_REV) RBMotor.direction_counter--;
  }
  else{
    if(RBMotor.direction_counter < BAD_HALL_COUNTER) RBMotor.direction_counter++;
  }
  RBMotor.direction = (RBMotor.direction_counter >= 0) ? 1 : -1;
  RBMotor.lastHallTriggered = 3;

  float new_value = (5714285.714f * RBMotor.direction) / (micros() - lastTime); //constant is degrees/interrupt *1000000us (360deg*(gearRatio/3)*1000000us) gear ratio = 1/21
  lastTime = micros();
  RBMotor.average_readings[RBMotor.int_pos] = new_value;
  if (RBMotor.int_pos++ > MOVING_AVG_SIZE) RBMotor.int_pos = 0;
}

//-----------------------------------------RIGHT FRONT MOTOR ISRS-----------------------------------------------//

void rf_hall_a_int(){
  if(RFMotor.lastHallTriggered == 2){
    if(RFMotor.direction_counter > BAD_HALL_COUNTER_REV) RFMotor.direction_counter--;
  }
  else{
    if(RFMotor.direction_counter < BAD_HALL_COUNTER) RFMotor.direction_counter++;
  }
  RFMotor.direction = (RFMotor.direction_counter >= 0) ? 1 : -1;
  RFMotor.lastHallTriggered = 1;

  float new_value = (5714285.714f * RFMotor.direction) / (micros() - lastTime); //constant is degrees/interrupt *1000000us (360deg*(gearRatio/3)*1000000us) gear ratio = 1/21
  lastTime = micros();
  RFMotor.average_readings[RFMotor.int_pos] = new_value;
  if (RFMotor.int_pos++ > MOVING_AVG_SIZE) RFMotor.int_pos = 0;
}

void rf_hall_b_int(){
  if(RFMotor.lastHallTriggered == 3){
    if(RFMotor.direction_counter > BAD_HALL_COUNTER_REV) RFMotor.direction_counter--;
  }
  else{
    if(RFMotor.direction_counter < BAD_HALL_COUNTER) RFMotor.direction_counter++;
  }
  RFMotor.direction = (RFMotor.direction_counter >= 0) ? 1 : -1;
  RFMotor.lastHallTriggered = 2;

  float new_value = (5714285.714f * RFMotor.direction) / (micros() - lastTime); //constant is degrees/interrupt *1000000us (360deg*(gearRatio/3)*1000000us) gear ratio = 1/21
  lastTime = micros();
  RFMotor.average_readings[RFMotor.int_pos] = new_value;
  if (RFMotor.int_pos++ > MOVING_AVG_SIZE) RFMotor.int_pos = 0;
}

void rf_hall_c_int(){
  if(RFMotor.lastHallTriggered == 1){
    if(RFMotor.direction_counter > BAD_HALL_COUNTER_REV) RFMotor.direction_counter--;
  }
  else{
    if(RFMotor.direction_counter < BAD_HALL_COUNTER) RFMotor.direction_counter++;
  }
  RFMotor.direction = (RFMotor.direction_counter >= 0) ? 1 : -1;
  RFMotor.lastHallTriggered = 3;

  float new_value = (5714285.714f * RFMotor.direction) / (micros() - lastTime); //constant is degrees/interrupt *1000000us (360deg*(gearRatio/3)*1000000us) gear ratio = 1/21
  lastTime = micros();
  RFMotor.average_readings[RFMotor.int_pos] = new_value;
  if (RFMotor.int_pos++ > MOVING_AVG_SIZE) RFMotor.int_pos = 0;
}



#endif //LEAVE THIS AT THE BOTTOM OF THIS FILE