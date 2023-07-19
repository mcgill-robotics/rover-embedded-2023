#ifdef SCIENCE //LEAVE THIS AT THE TOP OF THIS FILE

#include <Arduino.h>
#include "science.h"
#include "DRV8834.h"
#include "AccelStepper.h"
#include "CytronMotorDriver.h"
#include "ros.h"
#include "std_msgs/Float32MultiArray.h"

float scienceTargets[3];
float scienceFeedback[5];

AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);
IntervalTimer stepperTimer;

CytronMD MOT1(PWM_DIR, MOT1_PWM, MOT1_DIR);
CytronMD MOT2(PWM_DIR, MOT2_PWM, MOT2_DIR);

ros::NodeHandle nh;
std_msgs::Float32MultiArray scienceFBMsg;
std_msgs::Float32MultiArray scienceCmdMsg;
ros::Publisher scienceFB("scienceFB", &scienceFBMsg);

void scienceCB(const std_msgs::Float32MultiArray& input_msg){
  scienceTargets[0] = input_msg.data[0];
  scienceTargets[1] = input_msg.data[1];
  scienceTargets[2] = input_msg.data[2];
}

ros::Subscriber<std_msgs::Float32MultiArray> scienceCmd("scienceCmd", scienceCB);

void stepperISR(){
  stepper.run();
} 

void readAllMoistures(){
  scienceFeedback[0] = analogRead(MOIST_1) * (100.0/4095.0);
  scienceFeedback[1] = analogRead(MOIST_2) * (100.0/4095.0);
  scienceFeedback[2] = analogRead(MOIST_3) * (100.0/4095.0);
  scienceFeedback[3] = analogRead(MOIST_4) * (100.0/4095.0);
  scienceFeedback[4] = stepper.currentPosition() / GEAR_RATIO;
}

void writeAllMotors(){
  stepper.move(scienceTargets[0] * GEAR_RATIO);
  MOT1.setSpeed(scienceTargets[1]);
  MOT2.setSpeed(scienceTargets[2]);
}

void science_setup () {
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);

  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);

  stepper.setMaxSpeed(1000.0);
  stepper.setAcceleration(200.0);
  // stepper.moveTo(200);
  stepperTimer.begin(stepperISR, 1000000/STEPPER_FREQ);

  nh.initNode();

  scienceFBMsg.data = scienceFeedback;
  scienceFBMsg.data_length = 5;
  scienceCmdMsg.data_length = 3;

  nh.advertise(scienceFB);
  nh.subscribe(scienceCmd);

  nh.negotiateTopics();
}

void science_loop () {
  readAllMoistures();

  writeAllMotors();

  scienceFB.publish(&scienceFBMsg);
  nh.spinOnce();
  delay(5);
}



#endif //LEAVE THIS AT THE BOTTOM OF THIS FILE