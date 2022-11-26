#include <Arduino.h>
#include "encoder.h"

Encoder encoder(5, 4, 10, 3, 1.0f);

Encoder new_list[] = {encoder};


// The setup routine runs once when you press reset.
void setup() {
  Serial.begin(9600);  
}


// The loop routine runs over and over again forever.
void loop() {
  encoder.readEncoders(new_list);
  Serial.println(encoder.angular_position);
}
