#include "gps.h"
#include "TinyGPSPlus.h"

TinyGPSPlus gps;

float coords[2];

std_msgs::Float32MultiArray gpsMsg;
ros::NodeHandle nhGPS;
ros::Publisher pubGPS("roverGPSData", &gpsMsg);

void gps_setup(){
  Serial1.begin(9600);
  while(!Serial1);

  gpsMsg.data = coords;
  gpsMsg.data_length = 2;

  nhGPS.initNode();
  nhGPS.advertise(pubGPS);
}

void gps_loop(){

  coords[0] = gps.location.lat();
  coords[1] = gps.location.lng();

  while (Serial1.available())
    gps.encode(Serial1.read());
  pubGPS.publish(&gpsMsg);
  delay(5);

  nhGPS.spinOnce();
}