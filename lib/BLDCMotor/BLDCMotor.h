#ifndef BLDCMotor_h
#define BLDCMotor_h

#include <Arduino.h>
#include <Servo.h>
#include <arm_math.h>
#include "Deque.h"
//via library manager
#include "TeensyTimerInterrupt_Generic.h"

#define DIRECTIONBUFFER 15
#define SAMPLE_COUNT 2048
#define SAMPLING_FREQUENCY 8000
#define KERNSIZE 16

class WheelMotor
{
public:
    WheelMotor(uint8_t Pwmpin, uint8_t hallPinA, uint8_t hallPinB, uint8_t hallPinC);
    enum directions{FORWARDS=1, BACKWARDS=-1};

    float32_t target_speed = 0; //Target Speed of motor received by communications (Range is -100.0f to 100.0f)
    float32_t real_speed = 0; //Real speed of motor (in deg/s)
    double motor_us = 1500;
    volatile int direction = FORWARDS;
    volatile int currentAngle = 0;
    volatile int readingA, readingB, readingC = 0;
    volatile int directionCounter = 0;
    volatile float32_t max_freq = 0;

    void measureSpeed();
    void takeReading();
    void takeReading2();
    void takeReading3();
    void writeSpeed();
    void resetMotor();
    float32_t update(float32_t setpoint);

private:
    uint8_t hallAPin; //Pin of Hall Sensor A
    uint8_t hallBPin; //Pin of Hall Sensor B
    uint8_t hallCPin; //Pin of Hall Sensor C
    uint8_t PwmPin; //Pin of PWM connected to ESC

    Servo motor; //Servo object used to control the motor
    arm_rfft_fast_instance_f32 fft;
    arm_pid_instance_f32 pid;
    volatile uint32_t sample_count;
    volatile bool data_is_valid = false;
    // std::deque<float32_t> input_queue;
    Deque<float32_t, SAMPLE_COUNT> input_queue;
    Deque<float32_t, KERNSIZE+1> fft_output_queue;
    // std::deque<float32_t> fft_output_queue;
};

TeensyTimer ITimer(TEENSY_TIMER_1); 

float mapFloat(int x, float in_min, float in_max, float out_min, float out_max);
float32_t fourierTransform(float* input_queue, arm_rfft_fast_instance_f32 * fft);

#endif