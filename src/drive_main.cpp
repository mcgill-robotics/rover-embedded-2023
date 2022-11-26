#ifdef DRIVE //LEAVE THIS AT THE TOP OF THIS FILE

#include <Arduino.h>
#include "drive_main.h"
#include <encoder.h>

#define cs_pin PC_7
#define mosi_pin PA_12
#define miso_pin PA_11
#define sck_pin PB_3
Encoder encoder(cs_pin, mosi_pin, miso_pin, sck_pin);
Encoder *encoder_list = &encoder;

void drive_setup() {
  Serial.begin(9600);
  Encoder::resetEncoders(encoder_list);
}

void drive_loop() {
  Encoder::readEncoders(encoder_list);
  Serial.print("Position: ");
  Serial.print(encoder.angular_position);
  Serial.print(" Speed: ");
  Serial.println(encoder.angular_speed);
}

#endif //LEAVE THIS AT THE BOTTOM OF THIS FILE