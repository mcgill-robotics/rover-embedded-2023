// #ifdef POWER_SYS //LEAVE THIS AT THE TOP OF THIS FILE

#include "power.h"

#define CURRENT_SENSOR_CONSTANT 0.00201416015625 //3.3/(0.002 * 200.0 * 4096.0), PIN_VOLTAGE/(RESISTOR_VALUE * GAIN_CURRENT_SENSOR * ANALOG_RESOLUTION)

float currentsPower[8];
float angles[2];
int relays[4] = {0};
PWMServo servo1;
PWMServo servo2;

std_msgs::Float32MultiArray currentPowerMsg;
std_msgs::Float32MultiArray cmdMsg;
ros::NodeHandle nhPower;
ros::Publisher pubPower("currentPower", &currentPowerMsg);
ros::Subscriber<std_msgs::Float32MultiArray> subPower("powerCmd", msgCBPower);

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

  currentPowerMsg.data = currentsPower;
  currentPowerMsg.data_length = 8;
  cmdMsg.data_length = 6;

  nhPower.initNode();
  nhPower.advertise(pubPower);
  nhPower.subscribe(subPower);
}

void power_loop() {
  readAllCurrents();

  pubPower.publish(&currentPowerMsg);

  moveServos();
  writeRelays();

  nhPower.spinOnce();
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
  servo1.write((int) angles[0]);
  servo2.write((int) angles[1]);
}

void writeRelays(){
  digitalWrite(Drive_Pin, relays[0]);
  digitalWrite(Science_Pin, relays[1]);
  digitalWrite(Arm_12V_Pin, relays[2]);
  digitalWrite(Arm_24V_Pin, relays[3]);
}

void msgCBPower(const std_msgs::Float32MultiArray& input_msg){
  if(input_msg.data_length != 6) return;
  angles[0] = input_msg.data[0];
  angles[1] = input_msg.data[1];
  relays[0] = (int) input_msg.data[2];
  relays[1] = (int) input_msg.data[3];
  relays[2] = (int) input_msg.data[4];
  relays[3] = (int) input_msg.data[5];
}

// #endif //LEAVE THIS AT THE BOTTOM OF THIS FILE