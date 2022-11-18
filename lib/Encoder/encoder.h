#ifndef ENCODER_H
#define ENCODER_H
#include <Arduino.h>


class Encoder
{
public:
    Encoder(uint8_t cs_pin, float gear_ratio);
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
    void calculatePosition();
    void calculateSpeed();
    float gearRatio;
    float last_position;
    uint32_t lastTime = micros();
};

#endif //ENCODER_H