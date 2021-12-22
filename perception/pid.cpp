#include "pid.hpp"

Controller::PID::PID()
{

}

void Controller::PID::init(double dt, double kp, double ki, double kd, double saturation = 255) 
{
    dt_ = dt;
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    saturation_ = saturation;
}

double Controller::PID::calculate(double target, double feedback)
{
        double error = target - feedback;
        double Pout = error * kp_;
        double Dout = (error - prev_error)/dt_ * kd_;
        double Iout = 0;

        if(Pout + Dout < saturation_)
        {
            integral += error * dt_;
            Iout = integral * ki_;
        }
        
        prev_error = error;
        return Pout + Iout + Dout;
}
