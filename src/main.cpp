#include <Arduino.h>
#include "science.h"
#include "drive.h"
#include "killswitch.h"
#include "brushed_arm.h"
#include "brushless_arm.h"
#include "power.h"
#include "gps.h"
#include "ros.h"
#include "antenna.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "RoverArmMotor.h"

#define CONTROL_LOOP_PERIOD_US 5000

static unsigned long lastTime;
ros::NodeHandle nh;

void scienceCB(const std_msgs::Float32MultiArray &input_msg);
void powerCB(const std_msgs::Float32MultiArray &input_msg);
void panTiltAnglesCB(const std_msgs::Float32MultiArray &input_msg);
void armBrushedCB(const std_msgs::Float32MultiArray &input_msg);
void armBrushlessCB(const std_msgs::Float32MultiArray &input_msg);
void driveCB(const std_msgs::Float32MultiArray &input_msg);
void antennaCoordsCB(const std_msgs::Float32MultiArray &input_msg);
void roverCoordsCB(const std_msgs::Float32MultiArray &input_msg);

#ifdef SCIENCE
std_msgs::Float32MultiArray scienceFBMsg;
std_msgs::Float32MultiArray scienceCmdMsg;
ros::Publisher scienceFB("scienceFB", &scienceFBMsg);
ros::Subscriber<std_msgs::Float32MultiArray> scienceCmd("scienceCmd", scienceCB);
#endif
#ifdef POWER_SYS
std_msgs::Float32MultiArray powerFBMsg;
std_msgs::Float32MultiArray panTiltAnglesMsg;
std_msgs::Float32MultiArray powerCmdMsg;
ros::Publisher powerFB("powerFB", &powerFBMsg);
ros::Subscriber<std_msgs::Float32MultiArray> powerCmd("powerCmd", powerCB);
ros::Subscriber<std_msgs::Float32MultiArray> panTiltAngles("panTiltAngles", panTiltAnglesCB);
#endif
#ifdef GPS
std_msgs::Float32MultiArray roverGPSDataMsg;
ros::Publisher roverGPSData("roverGPSData", &roverGPSDataMsg);
#endif
#ifdef ANTENNA
std_msgs::Float32MultiArray roverGPSDataMsg;
std_msgs::Float32MultiArray antennaDataMsg;
ros::Subscriber<std_msgs::Float32MultiArray> roverGPSData("roverGPSData", roverCoordsCB);
ros::Subscriber<std_msgs::Float32MultiArray> antennaData("antennaData", antennaCoordsCB);
#endif
#ifdef USE_IMU
std_msgs::Float32MultiArray roverIMUDataMsg;
ros::Publisher roverIMUData("roverIMUData", &roverIMUDataMsg);
#endif
#ifdef KILLSWITCH
std_msgs::Float32MultiArray killswitchFBMsg;
ros::Publisher killswitchFB("killswitchFB", &killswitchFBMsg);
#endif
#if BRUSHED_ARM == 1
std_msgs::Float32MultiArray armBrushedFBMsg;
std_msgs::Float32MultiArray armBrushedCmdMsg;
ros::Publisher armBrushedFB("armBrushedFB", &armBrushedFBMsg);
ros::Subscriber<std_msgs::Float32MultiArray> armBrushedCmd("armBrushedCmd", armBrushedCB);
extern RoverArmMotor Wrist_Pitch;
#endif
#if BRUSHLESS_ARM == 1
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
  scienceFBMsg.data = scienceFeedback;
  scienceFBMsg.data_length = 5;
  scienceCmdMsg.data_length = 3;

  nh.advertise(scienceFB);
  nh.subscribe(scienceCmd);
#endif
#ifdef POWER_SYS
  power_setup();
  powerFBMsg.data = currentsPower;
  powerFBMsg.data_length = 8;
  powerCmdMsg.data_length = 4;
  panTiltAnglesMsg.data_length = 2;

  nh.advertise(powerFB);
  nh.subscribe(powerCmd);
  nh.subscribe(panTiltAngles);
#endif
#ifdef GPS
  gps_setup();
  roverGPSDataMsg.data = coords;
  roverGPSDataMsg.data_length = 2;
  nh.advertise(roverGPSData);
#endif
#ifdef USE_IMU
  roverIMUDataMsg.data = quaternion;
  roverIMUDataMsg.data_length = 6;

  nh.advertise(roverIMUData);
#endif
#ifdef ANTENNA
  roverGPSDataMsg.data = rover_coords;
  roverGPSDataMsg.data_length = 2;
  antennaDataMsg.data = antenna_positioning;
  antennaDataMsg.data_length = 3;

  nh.subscribe(roverGPSData);
  nh.subscribe(antennaData);
#endif
#ifdef KILLSWITCH
  killswitch_setup();
  killswitchFBMsg.data = currentsKS;
  killswitchFBMsg.data_length = 2;
  nh.advertise(killswitchFB);
#endif
#if BRUSHED_ARM == 1
  brushed_arm_setup();
  armBrushedFBMsg.data = armBrushedActualAngles;
  armBrushedFBMsg.data_length = 3;
  armBrushedCmdMsg.data_length = 3;

  nh.advertise(armBrushedFB);
  nh.subscribe(armBrushedCmd);
#endif
#if BRUSHLESS_ARM == 1
  brushless_arm_setup();
  armBrushlessFBMsg.data = armBrushlessActualAngles;
  armBrushlessFBMsg.data_length = 3;
  armBrushlessCmdMsg.data_length = 3;

  nh.advertise(armBrushlessFB);
  nh.subscribe(armBrushlessCmd);
#endif
#ifdef DRIVE
  drive_setup();
  driveFBMsg.data = realSpeeds;
  driveFBMsg.data_length = 4;
  driveCmdMsg.data_length = 4;

  nh.advertise(driveFB);
  nh.subscribe(driveCmd);
#endif

  nh.negotiateTopics();
  while (!nh.connected())
  {
    nh.negotiateTopics();
  }

  lastTime = micros();
}

void loop()
{
  while (micros() < lastTime + CONTROL_LOOP_PERIOD_US)
    ;
  lastTime += CONTROL_LOOP_PERIOD_US;

  // while (!nh.connected())
  // {
  //   nh.spinOnce();
  // }
  // nh.logdebug("Debug Statement");
  // nh.loginfo("Program info");
  // nh.logwarn("Warnings.");
  // nh.logerror("Errors..");
  // nh.logfatal("Fatalities!");

#ifdef SCIENCE
  science_loop();
  scienceFB.publish(&scienceFBMsg);
#endif
#ifdef POWER_SYS
  power_loop();
  powerFB.publish(&powerFBMsg);
#endif
#ifdef GPS
  gps_loop();
  roverGPSData.publish(&roverGPSDataMsg);
#endif
#ifdef ANTENNA
  antenna_loop();
#endif
#ifdef USE_IMU
  roverIMUData.publish(&roverIMUDataMsg);
#endif
#ifdef KILLSWITCH
  killswitch_loop();
  killswitchFB.publish(&killswitchFBMsg);
#endif
#if BRUSHED_ARM == 1
  brushed_arm_loop();
  // Wrist_Pitch.get_current_angle_sw(&Wrist_Pitch.currentAngle);
  armBrushedFB.publish(&armBrushedFBMsg);
#endif
#if BRUSHLESS_ARM == 1
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
  powerRelays[0] = (int)input_msg.data[0];
  powerRelays[1] = (int)input_msg.data[1];
  powerRelays[2] = (int)input_msg.data[2];
  powerRelays[3] = (int)input_msg.data[3];
}

void panTiltAnglesCB(const std_msgs::Float32MultiArray &input_msg)
{
  powerAngles[0] = input_msg.data[0];
  powerAngles[1] = input_msg.data[1];
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

void roverCoordsCB(const std_msgs::Float32MultiArray &input_msg)
{
  rover_coords[0] = input_msg.data[0];
  rover_coords[1] = input_msg.data[1];
}

void antennaCoordsCB(const std_msgs::Float32MultiArray &input_msg)
{
  antenna_positioning[0] = input_msg.data[0];
  antenna_positioning[1] = input_msg.data[1];
  antenna_positioning[2] = input_msg.data[2];
}