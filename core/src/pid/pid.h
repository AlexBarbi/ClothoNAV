#ifndef _PID_H_
#define _PID_H_

// Structure to hold PID parameters (Proportional, Integral, Derivative)
struct PIDParams {
    double Kp; // Proportional gain
    double Ki; // Integral gain
    double Kd; // Derivative gain
};

class PID
{
    public:
        PID( double dt, double max, double min, double Kp, double Kd, double Ki );
        ~PID();
        double calculate( double setpoint, double pv );
        void updateParams(PIDParams, double max, double min);

    public:
        void reset();
    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
        double _pre_pv;
};

  


PIDParams interpolatePIDParams(double speed);

PIDParams getPIDParamsForSpeed(double speed);
#endif