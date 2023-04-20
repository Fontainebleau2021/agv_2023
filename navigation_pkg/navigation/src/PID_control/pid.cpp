#include "pid.h"

namespace navigation
{
    PIDControl::PIDControl()
        : dt_(0.5), output_max_(1), output_min_(0), Kp_(0.1), Ki_(0.1), Kd_(0.1), pre_error_(0), integrator_(0)
    {
    }

    PIDControl::PIDControl(double Kp, double Ki, double Kd, double dt, double out_max, double out_min, double inter_max, double inter_min, double tau)
        : dt_(dt), output_max_(out_max), output_min_(out_min), Kp_(Kp), Ki_(Ki), Kd_(Kd), pre_error_(0),
          integrator_(0), differentiator_(0), inter_max_(inter_max), inter_min_(inter_min), tau_(tau)
    {
    }

    double PIDControl::CalculateOutput(double target, double cur_state)
    {
        double error = target - cur_state;

        double Kp_out = Kp_ * error;

        integrator_ += 0.5 * (error + pre_error_) * dt_;
        if (integrator_ > inter_max_)
        {
            integrator_ = inter_max_;
        }
        else if (integrator_ < inter_min_)
        {
            integrator_ = inter_min_;
        }

        double Ki_out = Ki_ * integrator_;

        differentiator_ = -(2.0 * Kd_ * (cur_state - pre_state_) /* Note: derivative on measurement, therefore minus sign in front of equation! */
                            + (2.0 * tau_ - dt_) * differentiator_) /
                          (2.0 * tau_ + dt_);

        // double Kd_out = Kd_ * derivative;

        double output = Kp_out + Ki_out + differentiator_;
        if (output > output_max_)
        {
            output = output_max_;
        }
        else if (output < output_min_)
        {
            output = output_min_;
        }
        pre_error_ = error;
        pre_state_ = cur_state;

        return output;
    }
} // namespace navigation
