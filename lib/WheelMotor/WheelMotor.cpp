#include <WheelMotor.h>

WheelMotor::WheelMotor(uint8_t Pwmpin, uint8_t hallPinA, uint8_t hallPinB, uint8_t hallPinC)
{
    PwmPin = Pwmpin;
    hallAPin = hallPinA;
    hallBPin = hallPinB;
    hallCPin = hallPinC;

    pinMode(hallAPin, INPUT);
    pinMode(hallBPin, INPUT);
    pinMode(hallCPin, INPUT);
    pinMode(PwmPin, OUTPUT);

    motor.attach(PwmPin);
    motor.writeMicroseconds(1500);

    movingAvg hallASmooth, hallBSmooth, hallCSmooth = new movingAvg(5);
    SimpleKalmanFilter hallKalmanFilter;
}