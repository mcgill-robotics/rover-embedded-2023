#ifdef POWER_SYS //LEAVE THIS AT THE TOP OF THIS FILE

#include "power.h"

#define CURRENT_SENSOR_CONSTANT 0.00201416015625 //3.3/(0.002 * 200.0 * 4096.0), PIN_VOLTAGE/(RESISTOR_VALUE * GAIN_CURRENT_SENSOR * ANALOG_RESOLUTION)

float currentsPower[8];
float powerAngles[2];
int powerRelays[4] = {0};
PWMServo servo1;
PWMServo servo2;

void power_setup() {
  // put your setup code here, to run once:
  // Initialize Pins
  pinMode(PWM_Servo_1_Pin, OUTPUT);
  pinMode(PWM_Servo_2_Pin, OUTPUT);
  pinMode(Arm_24V_Pin, OUTPUT);
  pinMode(Arm_12V_Pin, OUTPUT);
  pinMode(Arm24_Curr_Pin, INPUT);
  pinMode(Arm12_Curr_Pin, INPUT);
  pinMode(Drive_Pin, OUTPUT);
  pinMode(Science_Pin, OUTPUT);
  pinMode(Drive_Curr_1_Pin, INPUT);
  pinMode(Drive_Curr_2_Pin, INPUT);
  pinMode(Drive_Curr_3_Pin, INPUT);
  pinMode(Drive_Curr_4_Pin, INPUT);
  pinMode(Sci5_Curr_Pin, INPUT);
  pinMode(Sci12_Curr_Pin, INPUT);

  analogReadResolution(12);
  servo1.attach(PWM_Servo_1_Pin);
  servo2.attach(PWM_Servo_2_Pin);

  servo1.write(0);
  servo2.write(0);
}

void power_loop() {
  readAllCurrents();

  moveServos();

  writeRelays();
}

void readAllCurrents(){
  currentsPower[0] = analogRead(Drive_Curr_1_Pin) * CURRENT_SENSOR_CONSTANT;
  currentsPower[1] = analogRead(Drive_Curr_2_Pin) * CURRENT_SENSOR_CONSTANT;
  currentsPower[2] = analogRead(Drive_Curr_3_Pin) * CURRENT_SENSOR_CONSTANT;
  currentsPower[3] = analogRead(Drive_Curr_4_Pin) * CURRENT_SENSOR_CONSTANT;
  currentsPower[4] = analogRead(Arm12_Curr_Pin) * CURRENT_SENSOR_CONSTANT;
  currentsPower[5] = analogRead(Arm24_Curr_Pin) * CURRENT_SENSOR_CONSTANT;
  currentsPower[6] = analogRead(Sci12_Curr_Pin) * CURRENT_SENSOR_CONSTANT;
  currentsPower[7] = analogRead(Sci5_Curr_Pin) * CURRENT_SENSOR_CONSTANT;

}

void moveServos(){
  servo1.write((int) powerAngles[0]);
  servo2.write((int) powerAngles[1]);
}

void writeRelays(){
  digitalWrite(Drive_Pin, powerRelays[0]);
  digitalWrite(Science_Pin, powerRelays[1]);
  digitalWrite(Arm_12V_Pin, powerRelays[2]);
  digitalWrite(Arm_24V_Pin, powerRelays[3]);
}

#endif //LEAVE THIS AT THE BOTTOM OF THIS FILE