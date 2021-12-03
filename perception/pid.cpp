#include "pid.hpp"

Controller::PID::PID()
{

}

void Controller::PID::init(double dt, double kp, double ki, double kd)
{
    dt_ = dt;
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

double Controller::PID::calculate(double target, double feedback)
{
        double error = target - feedback;
        double Pout = error * kd_;
        integral += error * dt_;
        double Iout = integral * ki_;
        double Dout = (error - prev_error)/dt_ * kd_;
        prev_error = error;
        return Pout + Iout + Dout;
}