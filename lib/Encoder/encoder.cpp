//****************************************************************//
// Electrical Division, Mars Rover Team of McGill Robotics
// Authors: Vincent Boucher
// Fall 2022
// version: 1.0
//  
// AMT222C-V encoder code used in Arm system
// Functionality:
//            - Retrieve absolute angular position value through SPI
//            - Reset Encoder
//            - Calculate angular position and speed of output shaft
//****************************************************************//
#include <encoder.h>
#include <SPI.h>

static SPISettings encoderSettings(2000000, MSBFIRST, SPI_MODE0); //specs of our encoder

Encoder::Encoder(uint8_t cs_pin, float gear_ratio = 1.0f){
    chip_select = cs_pin;
    gearRatio = gear_ratio;
}

void Encoder::calculateSpeed(){
    angular_speed = (angular_position-last_position)/((micros()-lastTime)/1000000.0f);
    lastTime = micros();
}

void Encoder::calculatePosition(){
    float final_angle = (position/4095.0f) * 360.0f; //position argument's contribution to angle
    final_angle += 360.0f * turns; //add the turns's contribution to angle
    final_angle *= gearRatio; //transfer from input shaft to output shaft angle
    while(final_angle < 0 || final_angle > 360){
        final_angle += (final_angle < 0) ? 360.0f : -360.0f; //get the final_angle within 0-360 degrees
    }
    last_position = angular_position; //keep last value for speed calculation
    angular_position = final_angle; //set the Encoder object's position to the calculated angle
    calculateSpeed();
}

void Encoder::readEncoders(Encoder *encoder_list){
    int list_size = sizeof(encoder_list)/sizeof(encoder_list[0]);
    for(int i = 0;i<list_size;i++){
        uint8_t cs_pin = encoder_list[i].chip_select;
        SPI.beginTransaction(cs_pin, encoderSettings);
        bool parityCorrect = false;
        while(!parityCorrect){
            receive_buf = 0x00A00000; //command to read turns
            SPI.transfer(cs_pin, &receive_buf, 4);
            parityCorrect = checkbit((uint16_t*) &receive_buf) && checkbit((uint16_t*) &receive_buf+2);
        }
        encoder_list[i].turns = (int16_t) ((receive_buf && 0x3FFF0000)>>16);
        encoder_list[i].position = (uint16_t) ((receive_buf && 0x00003FFF)>>2); //position needs to be shifted right an extra 2 bits
        SPI.end();
        encoder_list[i].calculatePosition();
    }
}

void Encoder::resetEncoders(Encoder *encoder_list){
    int list_size = sizeof(encoder_list)/sizeof(encoder_list[0]);
    for(int i = 0;i<list_size;i++){
        uint8_t cs_pin = encoder_list[i].chip_select;
        SPI.beginTransaction(cs_pin, encoderSettings);
        SPI.transfer16(cs_pin, 0x0070);
    }
    SPI.end();
}

boolean Encoder::checkbit(uint16_t *input){
    uint16_t k0 = (*input && 0x00004000) >> 14; //checkbit 0 (even)
    uint16_t k1 = (*input && 0x00008000) >> 15; //checkbit 1 (odd)
    uint16_t odd_answer = (uint16_t) (countSetBits((unsigned int) *input && 0x00002AAA) + 1)%2;
    uint16_t even_answer = (uint16_t) (countSetBits((unsigned int) *input && 0x00001555) + 1)%2;
    if(k0 == even_answer && k1 == odd_answer){
        return true;
    }else{
        return false;
    }
}

/* Function to get no of set bits in binary
representation of positive integer n */
unsigned int countSetBits(unsigned int n)
{
    unsigned int count = 0;
    while (n) {
        count += n & 1;
        n >>= 1;
    }
    return count;
}