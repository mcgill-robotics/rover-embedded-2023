#include "gps.h"
#include "Quaternion.h"
#include "XimuReceiver.h"

TinyGPSPlus gps;
XimuReceiver imu;

float coords[2];
float quaternion[6];

void gps_setup(){
  Serial1.begin(9600);
  Serial2.begin(115200);
}

QuaternionStruct quatStruct;
Quaternion quat;
EulerAnglesStruct eulerStruct;

void gps_loop(){
  coords[0] = gps.location.lat();
  coords[1] = gps.location.lng();

  while (Serial1.available()) gps.encode(Serial1.read());
  while (Serial2.available()) imu.processNewChar(Serial2.read());

  if(imu.isQuaternionGetReady()) {
    quatStruct = imu.getQuaternion();
    quat = Quaternion(quatStruct.w, quatStruct.x, quatStruct.y, quatStruct.z);
    eulerStruct = quat.getEulerAngles();
    quaternion[0] = quatStruct.x;
    quaternion[1] = quatStruct.y;
    quaternion[2] = quatStruct.z;
    quaternion[3] = eulerStruct.roll;
    quaternion[4] = eulerStruct.pitch;
    quaternion[5] = eulerStruct.yaw;
  }
}