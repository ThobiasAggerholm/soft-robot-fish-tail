#ifndef __PID_CONTROLLER__HPP__
#define __PID_CONTROLLER__HPP__

namespace Controller
{
    class PID
    {
        public:
        PID();
        void init(double dt, double kp, double ki, double kd, double saturation = 255);

        double calculate(double target, double feedback);


        private:
        double prev_error = 0;
        double integral = 0;
        double dt_;
        double kp_;
        double ki_;
        double kd_;
        double saturation_ = 0;
    };
}


#endif