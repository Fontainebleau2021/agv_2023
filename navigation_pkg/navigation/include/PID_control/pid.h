#ifndef NAVI_PID_H
#define NAVI_PID_H

namespace navigation
{
    class PIDControl
    {
    public:
        PIDControl();
        PIDControl(double Kp, double Ki, double Kd, double dt, double out_max, double out_min, double inter_max, double inter_min, double tau);
        ~PIDControl() {}
        //根据当前值和目标值计算PID输出
        double CalculateOutput(double target, double cur_state);

    public:
        double output_max_;
        double output_min_;

        double inter_max_;
        double inter_min_;

        double dt_;
        double Kp_;
        double Ki_;
        double Kd_;
        double pre_error_;
        double pre_state_;

        double integrator_;
        double differentiator_;
        double tau_;
    };

    
} // namespace navigation

#endif // NAVI_PID_H