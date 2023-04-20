#ifndef NAVI_SEED_CONTROL_H
#define NAVI_SEED_CONTROL_H

#include "pid_ros.h"

#include <ros/ros.h>
#include <memory>
#include "pose2D.h"
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>

namespace navigation
{
    class SeedPIDControl
    {
    public:
        SeedPIDControl() = default;
        SeedPIDControl(std::string name);

        ~SeedPIDControl(){}

        // compute the twist msg output through pid
        double OmegaCalculate(double target, double cur);
        double VelCalculate(double target, double cur);

        // constrain delta theta
        void NormalizeTheta(double heading, double &theta);
        // judge current vehicle state (normal or left shift or right shift)
        double UpdateHeading(geometry_msgs::Pose &cur_pose, double heading);
        //设置机器人直线方向的倾角
        void SetHeading(double &heading) { heading_ = heading; }
        double Heading() { return heading_; }
        void SetStartPose(geometry_msgs::Pose &pose)
        {

            start_pose_ = pose;
        }
        //直线行驶控制输出
        void LineControlOutput(geometry_msgs::Pose &cur_pose, geometry_msgs::Twist &cur_vel, double heading);
        //原地旋转控制输出
        void RotateControlOutput(geometry_msgs::Pose &cur_pose, double target_heading, geometry_msgs::Twist &cur_vel);

    private:
        ros::NodeHandle nh_;

        PIDControlRos linear_vel_pid_;
        PIDControlRos omega_pid_;

        double target_vel_;
        double heading_;
        geometry_msgs::Pose start_pose_;

        bool enable_position_control_;
        double enable_position_distance_;
        double heading_tolerant_;
        double heading_offset_;
    };
    typedef std::shared_ptr<SeedPIDControl> SeedPIDControlPtr;
} // namespace navigation

#endif // NAVI_SEED_CONTROL_H