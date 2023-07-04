#include <Arduino.h>
#include "ros.h"
#include "std_msgs/Float32MultiArray.h"

#define B1a_A_Pin 1
#define B1a_B_Pin 2
#define B1a_C_Pin 3
#define B1a_D_Pin 4
#define B1a_BI_Pin 5
#define B1a_RBI_Pin 6
#define B1a_LT_Pin 15
#define B1b_A_Pin 7
#define B1b_B_Pin 8
#define B1b_C_Pin 9
#define B1b_D_Pin 10
#define B1b_LT_Pin 11
#define B1b_BI_Pin 12
#define B1b_RBI_Pin 14
#define CS_OUT1_Pin 17
#define CS_OUT2_Pin 16

void killswitch_setup();
void killswitch_loop();

void writeDisplays(float current);
void readCurrents();
