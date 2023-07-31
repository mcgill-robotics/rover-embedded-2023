#ifdef GPS

#include "gps.h"
#ifdef USE_IMU
#include "Quaternion.h"
#include "XimuReceiver.h"
#include "TinyGPSPlus.h"
#endif

TinyGPSPlus gps;
#ifdef USE_IMU
XimuReceiver imu;
float quaternion[6];
#endif

float coords[2];

void gps_setup(){
  Serial1.begin(9600);
  #ifdef USE_IMU
  Serial2.begin(115200);
  #endif
}

#ifdef USE_IMU
QuaternionStruct quatStruct;
Quaternion quat;
EulerAnglesStruct eulerStruct;
#endif

void gps_loop(){
  coords[0] = gps.location.lat();
  coords[1] = gps.location.lng();

  while (Serial1.available()) gps.encode(Serial1.read());
  #ifdef USE_IMU
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
  #endif
}

#endif