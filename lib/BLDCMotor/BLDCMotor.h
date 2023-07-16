#ifndef BLDCMotor_h
#define BLDCMotor_h

#include <Arduino.h>
#include <Servo.h>
#include <arm_math.h>
#include "Deque.h"
//via library manager
#include "TeensyTimerInterrupt_Generic.h"
#include "IntervalTimer.h"

#define DIRECTIONBUFFER 15
#define SAMPLE_COUNT 2048
#define SAMPLING_FREQUENCY 8000
#define HIGHEST_FREQUENCY 600
#define KERNSIZE 16
#define FORWARDS 1
#define BACKWARDS -1
#define PID_KP 0.35
#define PID_KI 0.0025
#define PID_KD 0

class WheelMotor
{
public:
    WheelMotor(uint8_t Pwmpin, uint8_t hallPinA, uint8_t hallPinB, uint8_t hallPinC);

    float32_t target_speed = 0; //Target Speed of motor received by communications (Range is -100.0f to 100.0f)
    float32_t real_speed = 0; //Real speed of motor (in deg/s)
    double motor_us = 1500;
    volatile int direction = FORWARDS;
    volatile int currentAngle = 0;
    volatile int readingA, readingB, readingC = 0;
    volatile int directionCounter = 0;
    // Deque<float32_t, SAMPLE_COUNT> input_queue;
    volatile float32_t max_freq = 0;

    void measureSpeed();
    void setTargetSpeed(float32_t setpoint);
    void takeReading();
    void writeSpeed();
    void resetMotor();
    float32_t update();

private:
    uint8_t hallAPin; //Pin of Hall Sensor A
    uint8_t hallBPin; //Pin of Hall Sensor B
    uint8_t hallCPin; //Pin of Hall Sensor C
    uint8_t PwmPin; //Pin of PWM connected to ESC

    Servo motor; //Servo object used to control the motor
    arm_rfft_fast_instance_f32 fft;
    arm_pid_instance_f32 pid;
    volatile bool data_is_valid = false;
    // Deque<float32_t, KERNSIZE+1> fft_output_queue;

    float32_t input_queue[SAMPLE_COUNT] = {0.0};
    int input_queue_iter = 0;
    float32_t fft_output_queue[KERNSIZE+1] = {0.0};
    int fft_output_queue_iter = 0;
}; 

float mapFloat(int x, float in_min, float in_max, float out_min, float out_max);

float32_t fourierTransform(float32_t input_queue[], arm_rfft_fast_instance_f32 * fft);

#endif