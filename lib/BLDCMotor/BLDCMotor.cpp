#include "BLDCMotor.h"

/// @brief Constructor for the WheelMotor object
/// @param Pwmpin  Pin connected to PWM of ESC
WheelMotor::WheelMotor(uint8_t Pwmpin, float* targetSpeed, float* realSpeed)
{
    PwmPin = Pwmpin;
    target_speed = targetSpeed;
    real_speed = realSpeed;

    pinMode(PwmPin, OUTPUT);

    motor.attach(PwmPin, 1100, 1900);
    writeSpeed();

    pid.Kp = PID_KP;
    pid.Ki = PID_KI;
    pid.Kd = PID_KD;

    arm_pid_init_f32(&pid, 1);
}

void WheelMotor::setTargetSpeed(float32_t setpoint){
    *target_speed = setpoint;
}

float32_t WheelMotor::update()
{
    *real_speed = (motor_us - 1500) / 4.0;

    float32_t err = *target_speed - *real_speed;

    float32_t output = arm_pid_f32(&pid, err);

    if(output > 100) output = 100;
    if(output < -100) output = -100;

    motor_us = output * 4 + 1500;

    return output;
}


void WheelMotor::writeSpeed(){
    int value = (int) motor_us;
    value = (value > 1900) ? 1900 : value;
    value = (value < 1100) ? 1100 : value;

    motor.writeMicroseconds(value);
}

float mapFloat(int x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void WheelMotor::resetMotor(){
    motor_us = 1500;
    writeSpeed();
}