#ifndef ENCODER_H
#define ENCODER_H
#include <Arduino.h>


class Encoder
{
public:
    Encoder(uint8_t cs_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t sclk_pin,float gear_ratio = 1.0f);
    uint16_t position;
    int16_t turns;
    float angular_position;
    float angular_speed;

    static void readEncoders(Encoder *encoder_list);
    static void resetEncoders(Encoder *encoder_list);
private:
    uint8_t chip_select;
    static uint32_t receive_buf;
    static boolean checkbit(uint16_t *input);
    static unsigned int countSetBits(unsigned int n);
    void calculatePosition();
    void calculateSpeed();
    float gearRatio;
    float last_position;
    uint32_t lastTime = micros();
};

#endif //ENCODER_H