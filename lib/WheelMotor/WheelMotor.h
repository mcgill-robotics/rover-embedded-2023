#include <Arduino.h>
#include <Servo.h>
#include <movingAvg.h>
#include <SimpleKalmanFilter.h>

class WheelMotor
{
public:
    WheelMotor(uint8_t Pwmpin, uint8_t hallPinA, uint8_t hallPinB, uint8_t hallPinC);

    volatile unsigned long hall_a_interrupts = 0;
    volatile unsigned long hall_b_interrupts = 0;
    volatile unsigned long hall_c_interrupts = 0;

private:
    uint8_t hallAPin;
    uint8_t hallBPin;
    uint8_t hallCPin;
    uint8_t PwmPin;
    unsigned long lastTime = micros();

    Servo motor;

    volatile int speed = -100;
    volatile int dir = 1;

    movingAvg hallASmooth(5);
    SimpleKalmanFilter hallKalmanFilter;
};