#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;

/**
 * Implementation
 */
PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0),
    _pre_pv(0)
{
}

double PID::calculate(double setpoint, double pv) {
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term with anti-windup
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative on measurement to avoid derivative kick
    double derivative = (_pre_pv - pv) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Clamp output and apply anti-windup
    if(output > _max) {
        output = _max;
        _integral = std::min(_integral, _max / _Ki);  // Anti-windup
    } else if(output < _min) {
        output = _min;
        _integral = std::max(_integral, _min / _Ki);  // Anti-windup
    }

    // Save error and PV for next calculation
    _pre_error = error;
    _pre_pv = pv;

    return output;
}

void PID::reset() {
    _integral = 0;
    _pre_error = 0;
    _pre_pv = 0;
}

void PID::updateParams(PIDParams params, double throttle_power, double brake_power) {
    _Kp = params.Kp;
    _Ki = params.Ki;
    _Kd = params.Kd;

    _max = throttle_power;
    _min = -brake_power;
}

PID::~PID()
{
}



#endif