// #ifdef KILLSWITCH //LEAVE THIS AT THE TOP OF THIS FILE

#include <Arduino.h>
#include "killswitch.h"
#include "ros.h"
#include "std_msgs/Float32MultiArray.h"

float currents[2];
std_msgs::Float32MultiArray currentsMsg;
ros::NodeHandle nh;
ros::Publisher pub("currentKS", &currentsMsg);

void killswitch_setup() {
  // Initialize pins
  pinMode(B1a_A_Pin, OUTPUT);
  pinMode(B1a_B_Pin, OUTPUT);
  pinMode(B1a_C_Pin, OUTPUT);
  pinMode(B1a_D_Pin, OUTPUT);
  pinMode(B1a_BI_Pin, OUTPUT);
  pinMode(B1a_RBI_Pin, OUTPUT);
  pinMode(B1a_LT_Pin, OUTPUT);
  pinMode(B1b_A_Pin, OUTPUT);
  pinMode(B1b_B_Pin, OUTPUT);
  pinMode(B1b_C_Pin, OUTPUT);
  pinMode(B1b_D_Pin, OUTPUT);
  pinMode(B1b_BI_Pin, OUTPUT);
  pinMode(B1b_RBI_Pin, OUTPUT);
  pinMode(B1b_LT_Pin, OUTPUT);
  pinMode(CS_OUT1_Pin, INPUT);
  pinMode(CS_OUT2_Pin, INPUT);

  // Set display pins HIGH
  digitalWrite(B1a_LT_Pin, HIGH);
  digitalWrite(B1a_BI_Pin, HIGH);
  digitalWrite(B1a_RBI_Pin, HIGH);
  digitalWrite(B1b_LT_Pin, HIGH);
  digitalWrite(B1b_BI_Pin, HIGH);
  digitalWrite(B1b_RBI_Pin, HIGH);

  analogReadResolution(12);

  currentsMsg.data = currents;
  nh.initNode();
  nh.advertise(pub);
}

void killswitch_loop() {
  readCurrents();

  writeDisplays(currents[0] + currents[1]);

  pub.publish(&currentsMsg);
  nh.spinOnce();
}

void writeDisplays(float current){
  uint8_t units = (int) abs(current) % 10;
  uint8_t tens = (int) abs(current) / 10;

  digitalWrite(B1a_A_Pin, (units & 0x01) >> 0);
  digitalWrite(B1a_B_Pin, (units & 0x02) >> 1);
  digitalWrite(B1a_C_Pin, (units & 0x04) >> 2);
  digitalWrite(B1a_D_Pin, (units & 0x08) >> 3);

  digitalWrite(B1b_A_Pin, (tens & 0x01) >> 0);
  digitalWrite(B1b_B_Pin, (tens & 0x02) >> 1);
  digitalWrite(B1b_C_Pin, (tens & 0x04) >> 2);
  digitalWrite(B1b_D_Pin, (tens & 0x08) >> 3);

}

void readCurrents(){
  int reading1 = analogRead(CS_OUT1_Pin);
  int reading2 = analogRead(CS_OUT2_Pin);

  //TODO: equations for currents and adding to array
}

// #endif //LEAVE THIS AT THE BOTTOM OF THIS FILE