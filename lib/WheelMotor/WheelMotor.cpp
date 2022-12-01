#include <WheelMotor.h>

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

    motor.attach(PwmPin);
    motor.writeMicroseconds(1500);
}

/// @brief Function to calculate the speed of the motor using the average_readings array.
/// @return Double of measured speed.
double WheelMotor::measureSpeed(){
    double sum = 0;
    for(int i=0;i<MOVING_AVG_SIZE;i++){
        sum += average_readings[i];
    }
    real_speed = sum / MOVING_AVG_SIZE;
    return real_speed;
}

/// @brief Function that writes the motor_us parameter to the motor
void WheelMotor::writeSpeed(){
    int value = (int) motor_us;
    value = (value > 1900) ? 1900 : value;
    value = (value < 1100) ? 1100 : value;

    motor.writeMicroseconds(value);
}