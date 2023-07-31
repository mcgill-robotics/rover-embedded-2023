#ifdef ANTENNA
#include "antenna.h"

float rover_coords[2]; //latitude, longitude
float antenna_positioning[3]; //latitude, longitude, compass angle

Servo servo;

void antenna_setup(){
    servo.attach(ANTENNA_PWM_PIN);
    delay(3000);
}

void antenna_loop(){
  // Calculating the differences
  double dLat = rover_coords[0] - antenna_positioning[0];
  double dLon = rover_coords[1] - antenna_positioning[1];

  // Converting to radians
  dLat = dLat * PI / 180.0;
  dLon = dLon * PI / 180.0;

  // Calculating the bearing
  double bearing_rad = atan2(dLon, dLat);
  double bearing_deg = bearing_rad * 180.0 / PI;

  // Normalizing the angle
  double bearing_norm = fmod((bearing_deg + 360.0), 360.0);

  // Adding the offset and setting the servo angle
  double servo_angle = bearing_norm + antenna_positioning[2];
  servo.write(servo_angle);
}

#endif