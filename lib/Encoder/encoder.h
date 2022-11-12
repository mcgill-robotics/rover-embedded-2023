#ifndef ENCODER_H
#define ENCODER_H
#include <Arduino.h>


class Encoder
{
public:
    Encoder(uint8_t cs_pin);
    uint16_t position;
    int16_t turns;

    static void readEncoders(Encoder *encoder_list);
    static void resetEncoders(Encoder *encoder_list);
private:
    uint8_t chip_select;
    static uint32_t receive_buf;
    static boolean checkbit(uint16_t *input);
};

#endif //ENCODER_H