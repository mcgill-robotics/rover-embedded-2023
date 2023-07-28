#include <Arduino.h>
#include "ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "TinyGPSPlus.h"

extern float coords[2];
extern float quaternion[6];

void gps_setup();
void gps_loop();