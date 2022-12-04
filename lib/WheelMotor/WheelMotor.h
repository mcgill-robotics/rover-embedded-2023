#include <Arduino.h>
#include <PWMServo.h>

#define MOVING_AVG_SIZE 30
#define FILTER_THRESHOLD 150
#define INT_CONSTANT 5342.914449f

/// @brief Instantiable class that represents a drive motor. Keeps track of speed and related variables.
class WheelMotor
{
public:
    WheelMotor(uint8_t Pwmpin, uint8_t hallPinA, uint8_t hallPinB, uint8_t hallPinC);

    volatile int lastHallTriggered = 1; //0 = init, 1 = HallA, 2 = HallB, 3 = HallC
    volatile int direction = 1; //default direction is always 1 (forwards)
    volatile int direction_counter = 0;
    volatile uint16_t int_pos = 0;
    volatile float average_readings[MOVING_AVG_SIZE] = {0.0f};
    double target_speed = 0; //Target Speed of motor received by communications (Range is -100.0f to 100.0f)
    double real_speed = 0; //Real speed of motor (in deg/s)
    volatile double motor_us = 1500;

    double measureSpeed();
    void writeSpeed();

private:
    uint8_t hallAPin; //Pin of Hall Sensor A
    uint8_t hallBPin; //Pin of Hall Sensor B
    uint8_t hallCPin; //Pin of Hall Sensor C
    uint8_t PwmPin; //Pin of PWM connected to ESC

    PWMServo motor; //Servo object used to control the motor
};