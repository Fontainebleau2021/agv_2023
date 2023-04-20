#ifndef NAVI_PID_ROS_H
#define NAVI_PID_ROS_H

#include <ros/ros.h>
#include "pid.h"

namespace navigation
{
    class PIDControlRos : public PIDControl
    {
    private:
        /* data */
        ros::NodeHandle nh_;

    public:
        PIDControlRos() = default;
        PIDControlRos(std::string name)
        {
            nh_ = ros::NodeHandle("~/" + name);
            //double Kp, Ki, Kd, dt, out_max, out_min, inter_max, inter_min, tau;
            nh_.param("dt", dt_, 0.8);
            nh_.param("Kp", Kp_, 0.8);
            nh_.param("Ki", Ki_, 0.8);
            nh_.param("Kd", Kd_, 0.8);
            nh_.param("output_max", output_max_, 0.8);
            nh_.param("output_min", output_min_, 0.8);
            nh_.param("inter_max", inter_max_, 0.8);
            nh_.param("inter_min", inter_min_, 0.8);
            nh_.param("tau", tau_, 0.01);
        }
        ~PIDControlRos() {}
    };



} // namespace navigation

#endif // NAVI_PID_ROS_H