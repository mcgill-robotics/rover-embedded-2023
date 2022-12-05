#include <Arduino.h>
#include "science_main.h"
#include "drive_main.h"
#include "sensor_main.h"
#include "upper_arm_main.h"
#include "lower_arm_main.h"
#include "power_main.h"
#include "Can.h"
#include "Rover_CAN_Defs.inl"

#define loop_hertz 100
#define CAN_WRITE_BUFFER_SIZE 100

CANBus::Message message_buffer[CAN_WRITE_BUFFER_SIZE] = {}; //buffer for keeping all the messages we want to send on the comms line
volatile int message_buffer_index = 0; //represents size of buffer, and used for index when adding to the buffer

CANBus::Message left_motors = {.id = (DRIVE_ID+LEFT_MOTORS_ID), .size=8};
CANBus::Message right_motors = {.id = (DRIVE_ID+RIGHT_MOTORS_ID), .size=8};

void CAN_write_load();
void CAN_read_decode(CANBus::Message &message);

void setup() {
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *canbus_timer = new HardwareTimer(Instance);

  canbus_timer->setOverflow(loop_hertz, HERTZ_FORMAT);
  canbus_timer->attachInterrupt(CAN_write_load);

  CANBus::init(CANBus::GPIOMode::A11A12, false);

  #ifdef SCIENCE
  CANBus::init_filter(0, 0, {.type = CANBus::FilterType::IDMask, .filters = {SCIENCE_ID}});
  science_setup();
  #endif
  #ifdef POWER
  CANBus::init_filter(0, 0, {.type = CANBus::FilterType::IDMask, .filters = {POWER_ID}});
  power_setup();
  #endif
  #ifdef SENSOR
  CANBus::init_filter(0, 0, {.type = CANBus::FilterType::IDMask, .filters = {SENSOR_ID}});
  sensor_setup();
  #endif
  #ifdef UPPER_ARM
  CANBus::init_filter(0, 0, {.type = CANBus::FilterType::IDMask, .filters = {ARM_ID}});
  upper_arm_setup();
  #endif
  #ifdef LOWER_ARM
  CANBus::init_filter(0, 0, {.type = CANBus::FilterType::IDMask, .filters = {ARM_ID}});
  lower_arm_setup();
  #endif
  #ifdef DRIVE
  CANBus::init_filter(0, 0, {.type = CANBus::FilterType::IDMask, .filters = {DRIVE_ID}});
  drive_setup();
  #endif
  CANBus::start();
  canbus_timer->resume();
}

void loop() {
  while(CANBus::getAvailableForRead(0) > 0){
    CANBus::Message output_message;
    CANBus::read(output_message, 0);
    CAN_read_decode(output_message);
  }

  #ifdef SCIENCE
  science_loop();
  #endif
  #ifdef POWER
  power_loop();
  #endif
  #ifdef SENSOR
  sensor_loop();
  #endif
  #ifdef UPPER_ARM
  upper_arm_loop();
  #endif
  #ifdef LOWER_ARM
  lower_arm_loop();
  #endif
  #ifdef DRIVE
  drive_loop();
  #endif
  //Could be changed to a IF instead of WHILE, depending on if we want to send all messages at once, or one at a time
  while(CANBus::getAvailableForWrite() > 0 && message_buffer_index > 0){
    CANBus::write(message_buffer[0]); //write the oldest message
    for(int i=0;i<message_buffer_index;i++){ //Shift elements left to account for removal of the first item
        message_buffer[i] = message_buffer[i+1];
    }
    message_buffer_index--; //decrement index
  }
}

void CAN_write_load(){
  if (message_buffer_index >= CAN_WRITE_BUFFER_SIZE) return;
  #ifdef SCIENCE
  //TODO
  #endif
  #ifdef POWER
  //TODO
  #endif
  #ifdef SENSOR
  //TODO
  #endif
  #ifdef UPPER_ARM
  //TODO
  #endif
  #ifdef LOWER_ARM
  //TODO
  #endif
  #ifdef DRIVE
  left_motors.data_32[0] = 0; //Put correct values here
  left_motors.data_32[1] = 0;
  right_motors.data_32[0] = 0; //Put correct values here
  right_motors.data_32[1] = 0;
  message_buffer[message_buffer_index++] = left_motors;
  message_buffer[message_buffer_index++] = right_motors;
  #endif
}

void CAN_read_decode(CANBus::Message &message){
  switch (message.id)
  {
  #ifdef SCIENCE
  //TODO
  #endif
  #ifdef POWER
  //TODO
  #endif
  #ifdef SENSOR
  //TODO
  #endif
  #ifdef UPPER_ARM
  //TODO
  #endif
  #ifdef LOWER_ARM
  //TODO
  #endif
  #ifdef DRIVE
  case DRIVE_ID+LEFT_MOTORS_ID:
    //fill in code
    break;
  case DRIVE_ID+RIGHT_MOTORS_ID:
    //fill in code
    break;
  #endif
  default:
    break;
  }
}