#include "BLDCMotor.h"
#include <algorithm>

const float bins_count = SAMPLING_FREQUENCY / SAMPLE_COUNT;
static float32_t fft_buffer_vector[SAMPLE_COUNT];
static float32_t mag_real[SAMPLE_COUNT / 2];
static float32_t mag[SAMPLE_COUNT];
static float32_t max_value;
static uint32_t max_index;


static const float32_t fir_kernel[KERNSIZE+1] = {
0.01680672, 0.02205882, 0.02731092, 0.03256303, 0.03781513, 0.04306723,
 0.04831933, 0.05357143, 0.05882353, 0.06407563, 0.06932773, 0.07457983,
 0.07983193, 0.08508403, 0.09033613, 0.09558824, 0.10084034};

/// @brief Constructor for the WheelMotor object
/// @param Pwmpin  Pin connected to PWM of ESC
/// @param hallPinA Pin connected to the Hall Sensor A
/// @param hallPinB Pin connected to the Hall Sensor B
/// @param hallPinC Pin connected to the Hall Sensor C
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

    motor.attach(PwmPin, 1100, 1900);
    writeSpeed();

    readingA = digitalRead(hallAPin);
    readingB = digitalRead(hallBPin);
    readingC = digitalRead(hallCPin);

    arm_rfft_fast_init_f32(&fft, SAMPLE_COUNT);

    pid.Kp = PID_KP;
    pid.Ki = PID_KI;
    pid.Kd = PID_KD;

    arm_pid_init_f32(&pid, 1);
}

/// @brief Function to calculate the speed of the motor using a fast fourier transform.
/// @return Double of measured speed.
void WheelMotor::measureSpeed(){
    // SerialUSB.println("HELLO");
    // if((*input_queue).get_size() >= 204){
    //     max_freq = fourierTransform((*input_queue).get_container(), &fft);
    //     SerialUSB.println(((*input_queue).get_size()));
    // }else{
    //     // SerialUSB.print("Measure: ");
    //     // SerialUSB.println((int) input_queue);
    // }
    max_freq = fourierTransform(fft_output_queue, &fft);
    SerialUSB.println(max_freq);
    real_speed = mapFloat(max_freq, 0.0f, HIGHEST_FREQUENCY, 0.0f, 100.0f);
    
    directionCounter = (real_speed == 0.0f) ? 0 : directionCounter;
    direction = (directionCounter >= 0) ? FORWARDS : BACKWARDS;
    real_speed *= direction;
    
    fft_output_queue[fft_output_queue_iter++] = real_speed;
    if(fft_output_queue_iter == KERNSIZE+1) fft_output_queue_iter = 0;
    // fft_output_queue.push_back(real_speed);
    // if(fft_output_queue.is_full()){fft_output_queue.pop_front();}

    float32_t accumulator = 0;
    // float32_t* container = fft_output_queue.get_container();
    for(int i=0; i<KERNSIZE+1; i++)
    {
        accumulator += (fft_output_queue[i] * fir_kernel[i]); 
    }

    real_speed = accumulator;
}

void WheelMotor::setTargetSpeed(float32_t setpoint){
    target_speed = setpoint;
}

float32_t WheelMotor::update()
{
    float32_t err = target_speed - real_speed;

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

float32_t fourierTransform(float32_t input_queue[], arm_rfft_fast_instance_f32 * fft){
    // (*timer).end();
    std::copy(input_queue, input_queue + SAMPLE_COUNT, fft_buffer_vector);

    // for(int i = 0; i < 100; i++){
    //     SerialUSB.print(input_queue[i]);
    //     SerialUSB.print(" ");
    // }
    // SerialUSB.println();

    // (*timer).begin();

    arm_rfft_fast_f32(fft, fft_buffer_vector, mag, 0);

    arm_cmplx_mag_f32(mag, mag_real, SAMPLE_COUNT/2);
    mag_real[0] = 0;

    arm_max_f32(mag_real, SAMPLE_COUNT/2, &max_value, &max_index);

    return max_index * bins_count;
}

float mapFloat(int x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void WheelMotor::takeReading(){

    // SerialUSB.println(input_queue.get_size());
    // SerialUSB.print("Reading: ");
    // SerialUSB.println((int) &input_queue);
    // float sum = 0;
    // for(int i = 0; i < 200; i++){
    //     sum += input_queue->get_value(i);
    // }
    // SerialUSB.println(sum);

    // if((*input_queue).get_size() >= 2048){
    //     (*input_queue).pop_front();
    //     data_is_valid = true;
    //     // SerialUSB.println("Data is valid");
    // }else{
    //     // SerialUSB.println(input_queue.get_size());
    // }

    input_queue[input_queue_iter++] = readingA;
    if(input_queue_iter == SAMPLE_COUNT) input_queue_iter = 0;

    // for(int i = 0; i < 100; i++){
    //     SerialUSB.print(input_queue[i]);
    //     SerialUSB.print(" ");
    // }
    // SerialUSB.println();
}

void WheelMotor::resetMotor(){
    motor_us = 1500;
    writeSpeed();
}