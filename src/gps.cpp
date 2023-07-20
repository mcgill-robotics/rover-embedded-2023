#include "gps.h"

TinyGPSPlus gps;

float coords[2];

void gps_setup(){
  Serial1.begin(9600);
  while(!Serial1);
}

void gps_loop(){
  coords[0] = gps.location.lat();
  coords[1] = gps.location.lng();

  while (Serial1.available()) gps.encode(Serial1.read());
}