/**
 * Copyright 2019 Bradley J. Snyder <snyder.bradleyj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"
#include <Arduino.h>
#include "rover_arm.h"
#include "RoverArmMotor.h"

extern ros::NodeHandle nh;

using namespace std;

class PIDImpl
{
public:
    PIDImpl(double dt, double angle_straight, double max, double min, double Kp, double Ki, double Kd);
    ~PIDImpl();
    double calculate(double setpoint, double pv);
    void setPID(double Kp, double Ki, double Kd);
    void use_gravity_compensation(bool gravity_compensation);
    double calculate_gravity_compensation(double angle_deg);

    // private:
    double _dt;
    double _max;
    double _min;
    double _Kp;
    double _Ki;
    double _Kd;
    double _pre_error;
    double _integral;
    double _angle_straight;
    bool _gravity_compensation;
};

PID::PID(double dt, double angle_straight, double max, double min, double Kp, double Ki, double Kd)
{
    pimpl = new PIDImpl(dt, angle_straight, max, min, Kp, Ki, Kd);
}

double PID::calculate(double setpoint, double pv)
{
    return pimpl->calculate(setpoint, pv);
}

PID::~PID()
{
    delete pimpl;
}

void PID::setPID(double Kp, double Ki, double Kd)
{
    pimpl->setPID(Kp, Ki, Kd);
}

void PID::use_gravity_compensation(bool gravity_compensation)
{
    pimpl->_gravity_compensation = gravity_compensation;
}

void PID::reset_integral()
{
    pimpl->_integral = 0;
}

/**
 * Implementation
 */
PIDImpl::PIDImpl(double dt, double angle_straight, double max, double min, double Kp, double Ki, double Kd) : _dt(dt),
                                                                                                              _angle_straight(angle_straight),
                                                                                                              _max(max),
                                                                                                              _min(min),
                                                                                                              _Kp(Kp),
                                                                                                              _Ki(Ki),
                                                                                                              _Kd(Kd),
                                                                                                              _pre_error(0),
                                                                                                              _integral(0),
                                                                                                              _gravity_compensation(false)
{
}

double PIDImpl::calculate(double setpoint, double pv)
{
    // Calculate gravity compensation
    double gravity_compensation = calculate_gravity_compensation(pv);

    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;
    double temp_Iout = abs(Iout);
    if (_integral > 0.0)
    {
        Iout = min(temp_Iout, 40.0);
    }
    else
    {
        Iout = -min(temp_Iout, 40.0);
    }

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout + gravity_compensation;

    // Restrict to max/min
    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    // Save error to previous error
    _pre_error = error;

    char buffer[256];
    sprintf(buffer,
            "Pout: %f, Iout: %f, Dout: %f, gravity_comp: %f, error: %f, _pre_error: %f, output: %f\r\n",
            Pout, Iout, Dout, gravity_compensation, error, _pre_error, output);
    nh.loginfo(buffer);

    return output;
}

PIDImpl::~PIDImpl()
{
}

void PIDImpl::setPID(double Kp, double Ki, double Kd)
{
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}

void PIDImpl::use_gravity_compensation(bool gravity_compensation)
{
    _gravity_compensation = gravity_compensation;
}

double PIDImpl::calculate_gravity_compensation(double angle_deg)
{
    // Constants
    const double arm_length = 0.5; // Length of arm in meters
    const double arm_mass = 1.0;   // Mass of arm in kilograms
    const double g = 9.81;         // Acceleration due to gravity in m/s^2

    // Calculate the angle in radians
    double angle_rad = (angle_deg - _angle_straight) * M_PI / 180.0;

    // Calculate the torque needed to counteract gravity
    // Torque = gravitational force * lever arm
    // The lever arm is arm_length/2 for a uniformly distributed mass
    double torque = arm_mass * g * arm_length * 0.5 * sin(angle_rad);

    return torque;
}

#endif