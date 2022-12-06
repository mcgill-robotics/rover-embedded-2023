#include "Can.h"
#include "Arduino.h"

void setup(){
    Serial.begin(9600);

    CANBus::init(CANBus::GPIOMode::A11A12, /* use_default_filter = */ true);

    // Filter which places all messages with ids { 0xde, 0xad, 0xbe, 0xef } into fifo 0.
    // #ifdef DRIVE
    // CANBus::init_filter(0, 0, { 
    //     .type = CANBus::FilterType::IDList, 
    //     .filters = { 1, 2, 3, 4 }});
    // #endif
    // #ifdef SCIENCE
    // CANBus::init_filter(0, 0, { 
    //     .type = CANBus::FilterType::IDList, 
    //     .filters = { 5, 6, 7, 8 }});
    // #endif

    // Filter which places all messages with ids { 0xfa, 0xce, 0xfe, 0xed } into fifo 1.
    // CANBus::init_filter(1, 1, { 
    //     .type = CANBus::FilterType::IDList, 
    //     .filters = { 0xfa, 0xce, 0xfe, 0xed }});

    CANBus::start();
}

void loop(){
    // checks if any messages from our first fifo (assigned to { 0xde, 0xad, 0xbe, 0xef } above) have arrived.
    if (CANBus::getAvailableForRead(0) > 0)
    {
        CANBus::Message m;
        CANBus::read(m, 0);
        Serial.print("Message Incoming: ");
        Serial.println(m.data_32[0]);
    }

    // checks if any messages from our second fifo (assigned to { 0xde, 0xad, 0xbe, 0xef } above) have arrived.
    if (CANBus::getAvailableForRead(1) > 0)
    {
        CANBus::Message m;
        CANBus::read(m, 1);
        Serial.print("Message Incoming: ");
        Serial.println(m.data_32[0]);
    }
   
   // checks if there are spaces available to transmit
   if (CANBus::getAvailableForWrite() > 0)
   {
        // Serial.println("CAN PRNT");
        #ifdef DRIVE
        CANBus::write({.id  = 5U, .size = 8, .data_32 = {1, 2}});
        #endif
        #ifdef SCIENCE
        CANBus::write({.id  = 1U, .size = 8, .data_32 = {3, 4}});
        #endif
   }

    Serial.println(CANBus::getAvailableForRead(0));
    delay(1000);
}