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

static unsigned long lastTime;
ros::NodeHandle nh;

void scienceCB(const std_msgs::Float32MultiArray &input_msg);
void powerCB(const std_msgs::Float32MultiArray &input_msg);
void armBrushedCB(const std_msgs::Float32MultiArray &input_msg);
void armBrushlessCB(const std_msgs::Float32MultiArray &input_msg);
void driveCB(const std_msgs::Float32MultiArray &input_msg);

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
#ifdef GPS
std_msgs::Float32MultiArray gpsMsg;
ros::Publisher pubGPS("roverGPSData", &gpsMsg);
#endif
#ifdef USE_IMU
std_msgs::Float32MultiArray imuMsg;
ros::Publisher pubIMU("roverIMUData", &imuMsg);
#endif
#ifdef KILLSWITCH
std_msgs::Float32MultiArray currentKSMsg;
ros::Publisher currentKS("currentKS", &currentKSMsg);
#endif
#ifdef BRUSHED_ARM
std_msgs::Float32MultiArray armBrushedFBMsg;
std_msgs::Float32MultiArray armBrushedCmdMsg;
ros::Publisher armBrushedFB("armBrushedFB", &armBrushedFBMsg);
ros::Subscriber<std_msgs::Float32MultiArray> armBrushedCmd("armBrushedCmd", armBrushedCB);
#endif
#ifdef BRUSHLESS_ARM
std_msgs::Float32MultiArray armBrushlessFBMsg;
std_msgs::Float32MultiArray armBrushlessCmdMsg;
ros::Publisher armBrushlessFB("armBrushlessFB", &armBrushlessFBMsg);
ros::Subscriber<std_msgs::Float32MultiArray> armBrushlessCmd("armBrushlessCmd", armBrushlessCB);
#endif
#ifdef DRIVE
std_msgs::Float32MultiArray driveFBMsg;
std_msgs::Float32MultiArray driveCmdMsg;
ros::Publisher driveFB("driveFB", &driveFBMsg);
ros::Subscriber<std_msgs::Float32MultiArray> driveCmd("driveCmd", driveCB);
#endif

void setup()
{
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
  gpsMsg.data = coords;
  gpsMsg.data_length = 2;

  nh.advertise(pubGPS);
#endif
#ifdef USE_IMU
  imuMsg.data = quaternion;
  imuMsg.data_length = 6;

  nh.advertise(pubIMU);
#endif
#ifdef KILLSWITCH
  killswitch_setup();
  currentKSMsg.data = currentsKS;
  currentKSMsg.data_length = 2;

  nh.advertise(currentKS);
#endif
#ifdef BRUSHED_ARM
  brushed_arm_setup();
  armBrushedFBMsg.data = armBrushedTargetAngles;
  armBrushedFBMsg.data_length = 3;
  armBrushedCmdMsg.data_length = 3;

  nh.advertise(armBrushedFB);
  nh.subscribe(armBrushedCmd);
#endif
#ifdef BRUSHLESS_ARM
  brushless_arm_setup();
  armBrushlessFBMsg.data = armBrushlessTargetAngles;
  armBrushlessFBMsg.data_length = 3;
  armBrushlessCmdMsg.data_length = 3;

  nh.advertise(armBrushlessFB);
  nh.subscribe(armBrushlessCmd);
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

void loop()
{
  while (micros() < lastTime + CONTROL_LOOP_PERIOD_US)
    ;
  lastTime += CONTROL_LOOP_PERIOD_US;

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
  pubGPS.publish(&gpsMsg);
#endif
#ifdef USE_IMU
  pubIMU.publish(&imuMsg);
#endif
#ifdef KILLSWITCH
  killswitch_loop();
  currentKS.publish(&currentKSMsg);
#endif
#ifdef BRUSHED_ARM
  brushed_arm_loop();
  armBrushedFB.publish(&armBrushedFBMsg);
#endif
#ifdef BRUSHLESS_ARM
  brushless_arm_loop();
  armBrushlessFB.publish(&armBrushlessFBMsg);
#endif
#ifdef DRIVE
  drive_loop();
  driveFB.publish(&driveFBMsg);
#endif

  nh.spinOnce();
}

void scienceCB(const std_msgs::Float32MultiArray &input_msg)
{
  scienceTargets[0] = input_msg.data[0];
  scienceTargets[1] = input_msg.data[1];
  scienceTargets[2] = input_msg.data[2];
}

void powerCB(const std_msgs::Float32MultiArray &input_msg)
{
  powerAngles[0] = input_msg.data[0];
  powerAngles[1] = input_msg.data[1];
  powerRelays[0] = (int)input_msg.data[2];
  powerRelays[1] = (int)input_msg.data[3];
  powerRelays[2] = (int)input_msg.data[4];
  powerRelays[3] = (int)input_msg.data[5];
}

void armBrushedCB(const std_msgs::Float32MultiArray &input_msg)
{
  armBrushedTargetAngles[0] = input_msg.data[0];
  armBrushedTargetAngles[1] = input_msg.data[1];
  armBrushedTargetAngles[2] = input_msg.data[2];
}
void armBrushlessCB(const std_msgs::Float32MultiArray &input_msg)
{
  armBrushlessTargetAngles[0] = input_msg.data[0];
  armBrushlessTargetAngles[1] = input_msg.data[1];
  armBrushlessTargetAngles[2] = input_msg.data[2];
}
void driveCB(const std_msgs::Float32MultiArray &input_msg)
{
  targetSpeeds[0] = input_msg.data[0];
  targetSpeeds[1] = input_msg.data[1];
  targetSpeeds[2] = input_msg.data[2];
  targetSpeeds[3] = input_msg.data[3];
}