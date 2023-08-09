#ifndef BLDCMotor_h
#define BLDCMotor_h

#include <Arduino.h>
#include <Servo.h>
#include <arm_math.h>

#define FORWARDS 1
#define BACKWARDS -1
#define PID_KP 0.9
#define PID_KI 0.0025
#define PID_KD 0

class WheelMotor
{
public:
    WheelMotor(uint8_t Pwmpin, float* targetSpeed, float* realSpeed);

    float32_t* target_speed; //Target Speed of motor received by communications (Range is -100.0f to 100.0f)
    float32_t* real_speed; //Real speed of motor (in deg/s)
    double motor_us = 1500;
    volatile int direction = FORWARDS;

    void setTargetSpeed(float32_t setpoint);
    void writeSpeed();
    void resetMotor();
    float32_t update();

private:
    uint8_t PwmPin; //Pin of PWM connected to ESC

    Servo motor; //Servo object used to control the motor
    arm_pid_instance_f32 pid;
}; 

float mapFloat(int x, float in_min, float in_max, float out_min, float out_max);

#endif