#include "seed_control.h"


namespace navigation
{
    SeedPIDControl::SeedPIDControl(std::string name)
    {
        nh_ = ros::NodeHandle("~/" + name);
        linear_vel_pid_ = PIDControlRos("seed_linear_vel_pid");
        omega_pid_ = PIDControlRos("seed_omega_pid");
        ROS_INFO("kp: %f", linear_vel_pid_.Kp_);
        nh_.param("target_vel", target_vel_, 0.4);

        nh_.param("line_heading", heading_, 0.0);
        nh_.param("enable_position_control", enable_position_control_, false);
        nh_.param("enable_position_distance", enable_position_distance_, 10.0);
        nh_.param("heading_tolerant", heading_tolerant_, 0.02);
        nh_.param("heading_offset", heading_offset_, 0.08);
    }

    double SeedPIDControl::OmegaCalculate(double target, double cur)
    {
        NormalizeTheta(target, cur);
        return omega_pid_.CalculateOutput(target, cur);
    }

    double SeedPIDControl::VelCalculate(double target, double cur)
    {
        return linear_vel_pid_.CalculateOutput(target, cur);
    }


    void SeedPIDControl::NormalizeTheta(double heading, double &theta)
    {

        if ((heading - theta) > M_PI)
        {
            theta += 2 * M_PI;
        }
        else if ((heading - theta) < -M_PI)
        {
            theta -= 2 * M_PI;
        }
    }

    double SeedPIDControl::UpdateHeading(geometry_msgs::Pose &cur_pose, double heading)
    {
        heading_ = heading;
        ROS_INFO("start pose x: %f y: %f",start_pose_.position.x, start_pose_.position.y);
        double delta_x = cur_pose.position.x - start_pose_.position.x;
        double delta_y = cur_pose.position.y - start_pose_.position.y;
        double dis = hypot(delta_x, delta_y);
        if (hypot(delta_x, delta_y) < enable_position_distance_)
        {
            ROS_INFO("NOW is just velcity and theta control");
            return heading_;
        }
        double slope = atan2(delta_y, delta_x);
        ROS_INFO("slope from cur pose to start pose: %f", slope);
        NormalizeTheta(heading_, slope);
        if (abs(slope - heading_) < heading_tolerant_)
        {
            return heading_;
        }
        else if ((slope - heading_) > heading_tolerant_)
        {
            return heading_ - heading_offset_;
        }
        else
        {
            return heading_ + heading_offset_;
        }
    }


    void SeedPIDControl::LineControlOutput(geometry_msgs::Pose &cur_pose, geometry_msgs::Twist &cur_vel, double heading)
    {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(cur_pose.orientation, quat);
        double roll, pitch, cur_theta;
        tf::Matrix3x3(quat).getRPY(roll, pitch, cur_theta); //进行转换
        double target_heading = UpdateHeading(cur_pose, heading);
        double omega = OmegaCalculate(target_heading, cur_theta);
        double vel = VelCalculate(target_vel_, cur_vel.linear.x);
        cur_vel.linear.x = vel;
        cur_vel.angular.z = omega;
    }

    void SeedPIDControl::RotateControlOutput(geometry_msgs::Pose &cur_pose, double target_heading, geometry_msgs::Twist &cur_vel)
    {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(cur_pose.orientation, quat);
        double roll, pitch, cur_theta;
        tf::Matrix3x3(quat).getRPY(roll, pitch, cur_theta); //进行转换
        NormalizeTheta(target_heading, cur_theta);
        double omega = OmegaCalculate(target_heading, cur_theta);
        cur_vel.linear.x = 0;
        cur_vel.angular.z = omega;
        SetStartPose(cur_pose);
    }




    
} // namespace navigation
