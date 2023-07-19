#include <Arduino.h>
#include "science.h"
#include "drive.h"
#include "killswitch.h"
#include "brushed_arm.h"
#include "brushless_arm.h"
#include "power.h"
#include "gps.h"
#include "ros.h"
#include "std_msgs/Float32MultiArray.h"

#define CONTROL_LOOP_PERIOD_US 5000

unsigned long lastTime;
ros::NodeHandle nh;

void scienceCB(const std_msgs::Float32MultiArray& input_msg);
void powerCB(const std_msgs::Float32MultiArray& input_msg);
void arm12CB(const std_msgs::Float32MultiArray& input_msg);
void arm24CB(const std_msgs::Float32MultiArray& input_msg);
void driveCB(const std_msgs::Float32MultiArray& input_msg);
void reconnect();

#ifdef SCIENCE
std_msgs::Float32MultiArray scienceFBMsg;
std_msgs::Float32MultiArray scienceCmdMsg;
ros::Publisher scienceFB("scienceFB", &scienceFBMsg);
ros::Subscriber<std_msgs::Float32MultiArray> scienceCmd("scienceCmd", scienceCB);
#endif
#ifdef POWER_SYS
std_msgs::Float32MultiArray currentPowerMsg;
std_msgs::Float32MultiArray powerCmdMsg;
ros::Publisher currentPower("currentPower", &currentPowerMsg);
ros::Subscriber<std_msgs::Float32MultiArray> powerCmd("powerCmd", powerCB);
#endif
#ifdef KILLSWITCH
std_msgs::Float32MultiArray currentKSMsg;
ros::Publisher currentKS("currentKS", &currentKSMsg);
#endif
#ifdef BRUSHED_ARM
std_msgs::Float32MultiArray arm12FBMsg;
std_msgs::Float32MultiArray arm12CmdMsg;
ros::Publisher arm12FB("arm12FB", &arm12FBMsg);
ros::Subscriber<std_msgs::Float32MultiArray> arm12Cmd("arm12Cmd", arm12CB);
#endif
#ifdef BRUSHLESS_ARM
std_msgs::Float32MultiArray arm24FBMsg;
std_msgs::Float32MultiArray arm24CmdMsg;
ros::Publisher arm24FB("arm24FB", &arm24FBMsg);
ros::Subscriber<std_msgs::Float32MultiArray> arm24Cmd("arm24Cmd", arm24CB);
#endif
#ifdef DRIVE
std_msgs::Float32MultiArray driveFBMsg;
std_msgs::Float32MultiArray driveCmdMsg;
ros::Publisher driveFB("driveFB", &driveFBMsg);
ros::Subscriber<std_msgs::Float32MultiArray> driveCmd("driveCmd", driveCB);
#endif

void setup() {
  nh.initNode();
  #ifdef SCIENCE
  science_setup();
  scienceFBMsg.data = moistures;
  scienceFBMsg.data_length = 4;
  scienceCmdMsg.data_length = 3;

  nh.advertise(scienceFB);
  nh.subscribe(scienceCmd);
  #endif
  #ifdef POWER_SYS
  power_setup();
  currentPowerMsg.data = currentsPower;
  currentPowerMsg.data_length = 8;
  powerCmdMsg.data_length = 6;

  nh.advertise(currentPower);
  nh.subscribe(powerCmd);
  #endif
  #ifdef GPS
  gps_setup();
  #endif
  #ifdef KILLSWITCH
  killswitch_setup();
  currentKSMsg.data = currentsKS;
  currentKSMsg.data_length = 2;

  nh.advertise(currentKS);
  #endif
  #ifdef BRUSHED_ARM
  brushed_arm_setup();
  arm12FBMsg.data = arm12TargetAngles;
  arm12FBMsg.data_length = 3;
  arm12CmdMsg.data_length = 3;

  nh.advertise(arm12FB);
  nh.subscribe(arm12Cmd);
  #endif
  #ifdef BRUSHLESS_ARM
  brushless_arm_setup();
  arm24FBMsg.data = arm24TargetAngles;
  arm24FBMsg.data_length = 3;
  arm24CmdMsg.data_length = 3;

  nh.advertise(arm24FB);
  nh.subscribe(arm24Cmd);
  #endif
  #ifdef DRIVE
  drive_setup();
  driveFBMsg.data = targetSpeeds;
  driveFBMsg.data_length = 3;
  driveCmdMsg.data_length = 3;

  nh.advertise(driveFB);
  nh.subscribe(driveCmd);
  #endif

  nh.negotiateTopics();

  lastTime = micros();
}

void loop() {
  while(micros() < lastTime + CONTROL_LOOP_PERIOD_US);
  lastTime += CONTROL_LOOP_PERIOD_US;

  // if(!nh.connected()){
  //   while(!nh.connected()){
  //     nh.spinOnce();
  //   }
  //   reconnect();
  // }

  #ifdef SCIENCE
  science_loop();
  scienceFB.publish(&scienceFBMsg);
  #endif
  #ifdef POWER_SYS
  power_loop();
  currentPower.publish(&currentPowerMsg);
  #endif
  #ifdef GPS
  gps_loop();
  #endif
  #ifdef KILLSWITCH
  killswitch_loop();
  currentKS.publish(&currentKSMsg);
  #endif
  #ifdef BRUSHED_ARM
  brushed_arm_loop();
  arm12FB.publish(&arm12FBMsg);
  #endif
  #ifdef BRUSHLESS_ARM
  brushless_arm_loop();
  arm24FB.publish(&arm24FBMsg);
  #endif
  #ifdef DRIVE
  drive_loop();
  driveFB.publish(&driveFBMsg);
  #endif

  nh.spinOnce();
}

void scienceCB(const std_msgs::Float32MultiArray& input_msg){
  scienceTargets[0] = input_msg.data[0];
  scienceTargets[1] = input_msg.data[1];
  scienceTargets[2] = input_msg.data[2];
}

void powerCB(const std_msgs::Float32MultiArray& input_msg){
  powerAngles[0] = input_msg.data[0];
  powerAngles[1] = input_msg.data[1];
  powerRelays[0] = (int) input_msg.data[2];
  powerRelays[1] = (int) input_msg.data[3];
  powerRelays[2] = (int) input_msg.data[4];
  powerRelays[3] = (int) input_msg.data[5];
}

void arm12CB(const std_msgs::Float32MultiArray& input_msg){
  arm12TargetAngles[0] = input_msg.data[0];
  arm12TargetAngles[1] = input_msg.data[1];
  arm12TargetAngles[2] = input_msg.data[2];
}
void arm24CB(const std_msgs::Float32MultiArray& input_msg){
  arm24TargetAngles[0] = input_msg.data[0];
  arm24TargetAngles[1] = input_msg.data[1];
  arm24TargetAngles[2] = input_msg.data[2];
}
void driveCB(const std_msgs::Float32MultiArray& input_msg){
  targetSpeeds[0] = input_msg.data[0];
  targetSpeeds[1] = input_msg.data[1];
  targetSpeeds[2] = input_msg.data[2];
  targetSpeeds[3] = input_msg.data[3];
}

void reconnect(){
  nh.initNode();
  #ifdef SCIENCE
  nh.advertise(scienceFB);
  nh.subscribe(scienceCmd);
  #endif
  #ifdef POWER_SYS
  nh.advertise(currentPower);
  nh.subscribe(powerCmd);
  #endif
  #ifdef GPS
  nh.advertise(pubGPS);
  #endif
  #ifdef KILLSWITCH
  nh.advertise(currentKS);
  #endif
  #ifdef BRUSHED_ARM
  nh.advertise(arm12FB);
  nh.subscribe(arm12Cmd);
  #endif
  #ifdef BRUSHLESS_ARM
  nh.advertise(arm24FB);
  nh.subscribe(arm24Cmd);
  #endif
  #ifdef DRIVE
  nh.advertise(driveFB);
  nh.subscribe(driveCmd);
  #endif

  nh.negotiateTopics();
  lastTime = micros() + CONTROL_LOOP_PERIOD_US;
}