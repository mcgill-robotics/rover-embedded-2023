﻿/**
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
    PIDImpl(double dt, double max, double min, double Kp, double Ki, double Kd);
    ~PIDImpl();
    double calculate(double setpoint, double pv);
    void setPID(double Kp, double Ki, double Kd);

    // private:
    double _dt;
    double _max;
    double _min;
    double _Kp;
    double _Ki;
    double _Kd;
    double _pre_error;
    double _integral;
};

PID::PID(double dt, double max, double min, double Kp, double Ki, double Kd)
{
    pimpl = new PIDImpl(dt, max, min, Kp, Ki, Kd);
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
    pimpl->_Kp = Kp;
    pimpl->_Ki = Ki;
    pimpl->_Kd = Kd;
}

void PID::reset_integral()
{
    pimpl->_integral = 0;
}

/**
 * Implementation
 */
PIDImpl::PIDImpl(double dt, double max, double min, double Kp, double Ki, double Kd) : _dt(dt),
                                                                                       _max(max),
                                                                                       _min(min),
                                                                                       _Kp(Kp),
                                                                                       _Ki(Ki),
                                                                                       _Kd(Kd),
                                                                                       _pre_error(0),
                                                                                       _integral(0)
{
}

double PIDImpl::calculate(double setpoint, double pv)
{

    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;
    Iout = min(Iout, 70.0);

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // double derivative = error / _dt;
    // double Dout = _Kd * derivative / 100;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    // Save error to previous error
    _pre_error = error;

    char buffer[256];
    sprintf(buffer,
            "Pout: %f, Iout: %f, Dout: %f, error: %f, _pre_error: %f, output: %f\r\n",
            Pout, Iout, Dout, error, _pre_error, output);
    nh.loginfo(buffer);

    return output;
}

PIDImpl::~PIDImpl()
{
}
void PIDImpl::setPID(double Kp, double Ki, double Kd)
{
    Serial.printf("Kp: %f, Kd: %f, Ki: %f\r\n", Kp, Ki, Kd);
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}

#endif