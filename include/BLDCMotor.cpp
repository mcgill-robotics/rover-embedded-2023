#include "BLDCMotor.h"
#include <algorithm>

const float bins_count = SAMPLING_FREQUENCY / SAMPLE_COUNT;
static float32_t fft_buffer_vector[SAMPLE_COUNT];
static float32_t fir_buffer_vector[KERNSIZE + 1];
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
    motor_us = 1500;
    writeSpeed();

    readingA = digitalRead(hallAPin);
    readingB = digitalRead(hallBPin);
    readingC = digitalRead(hallCPin);

    arm_rfft_fast_init_f32(&fft, SAMPLE_COUNT);

    pid.Kp = 0.35;
    pid.Ki = 0.0025;
    pid.Kd = 0;

    arm_pid_init_f32(&pid, 1);
}

/// @brief Function to calculate the speed of the motor using a fast fourier transform.
/// @return Double of measured speed.
void WheelMotor::measureSpeed(){
    // Serial.println("Entering FFT");
    //if the data is good use the FFT to find the dominant frequency 
    if(data_is_valid) max_freq = fourierTransform(input_queue.get_container(), &fft);
    real_speed = mapFloat(max_freq, 0.0f, 600.0f, 0.0f, 100.0f);
    // Serial.println("Exited FFT");
    
    directionCounter = (real_speed == 0.0f) ? 0 : directionCounter;
    direction = (directionCounter >= 0) ? FORWARDS : BACKWARDS;
    real_speed *= direction;

    // Serial.println("Entering ftt_output_queue_push");
    fft_output_queue.push_back(real_speed);
    if(fft_output_queue.is_full()){fft_output_queue.pop_front();}
    // Serial.println("Exited ftt_output_queue_push");

    // Serial.println("Entering Kalman filter");
    float32_t accumulator = 0;
    float32_t* container = fft_output_queue.get_container();
    for(int i=0; i<fft_output_queue.get_size(); i++)
    {
        accumulator += (container[i] * fir_kernel[i]); 
    }

    real_speed = accumulator;
    // Serial.println("Exiting Kalman filter");

}

float32_t WheelMotor::update(float32_t setpoint)
{
    //Serial.println("update1");
    float32_t us_current_speed = 4 * real_speed + 1500.0f;
    float32_t err = (setpoint - us_current_speed);

    // Serial.print(err);
    // Insert PID here
    float32_t output = arm_pid_f32(&pid, err);
    //Serial.println("update2");

    if(output > 400) output = 400;
    if(output < -400) output = -400;

    output += 1500;

    // Serial.print(",");
    // Serial.println(output);

    //Serial.println("update3");

    return output;

}


/// @brief Function that writes the motor_us parameter to the motor
void WheelMotor::writeSpeed(){
    int value = (int) motor_us;
    value = (value > 1900) ? 1900 : value;
    value = (value < 1100) ? 1100 : value;

    motor.writeMicroseconds(value);
}

float32_t fourierTransform(float32_t input_queue[SAMPLE_COUNT], arm_rfft_fast_instance_f32* fft){

    // Serial.println("Entering input_queue buffer copy");
    // Pause the timer interrupt so no new samples are added to the queue when copying
    ITimer.stopTimer();
    //Copy SAMPLE_COUNT number of samples from the input data queue into the fft_buffer
    std::copy(input_queue, input_queue + SAMPLE_COUNT, fft_buffer_vector);
    ITimer.restartTimer();

    // Serial.println("Exiting input_queue buffer copy");

    // Serial.println("Entering ARM FFT function");

    arm_rfft_fast_f32(fft, fft_buffer_vector, mag, 0);
    // Serial.println("Exiting ARM FFT function");

     // Serial.println("Entering ARM cplxmag function");
    arm_cmplx_mag_f32(mag, mag_real, SAMPLE_COUNT/2);
    mag_real[0] = 0;
    // mag_real[SAMPLE_COUNT/2] = 0;
    // Serial.println("Exiting ARM cplxmag function");

    // Serial.println("Entering ARM max function");
    arm_max_f32(mag_real, SAMPLE_COUNT/2, &max_value, &max_index);
    // Serial.println("Exiting ARM max function");

    return max_index * bins_count;
}

float mapFloat(int x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void WheelMotor::takeReading(){

    if(input_queue.is_full()){
        input_queue.pop_front();
        data_is_valid = true;
    }
    input_queue.push_back((float32_t) readingA);
    sample_count++;

    // if(input_queue.size() == SAMPLE_COUNT){
    //     data_is_valid = true;
    // }else if(input_queue.size() > SAMPLE_COUNT){
    //     input_queue.pop_front();
    // }else{
    //     data_is_valid = false;
    // }
}

void WheelMotor::takeReading2(){
    readingB = digitalRead(hallBPin);
    if(input_queue.is_full()){
        input_queue.pop_front();
        data_is_valid = true;
    }
    input_queue.push_back((float32_t) readingB);
    sample_count++;

    // if(input_queue.size() == SAMPLE_COUNT){
    //     data_is_valid = true;
    // }else if(input_queue.size() > SAMPLE_COUNT){
    //     input_queue.pop_front();
    // }else{
    //     data_is_valid = false;
    // }
}

void WheelMotor::takeReading3(){
    readingC = digitalRead(hallCPin);
    if(input_queue.is_full()){
        input_queue.pop_front();
        data_is_valid = true;
    }
    input_queue.push_back((float32_t) readingC);
    sample_count++;

    // if(input_queue.size() == SAMPLE_COUNT){
    //     data_is_valid = true;
    // }else if(input_queue.size() > SAMPLE_COUNT){
    //     input_queue.pop_front();
    // }else{
    //     data_is_valid = false;
    // }
}

void WheelMotor::resetMotor(){
    motor_us = 1500;
    writeSpeed();
}