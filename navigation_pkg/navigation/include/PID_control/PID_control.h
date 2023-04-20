#ifndef LINE_PID_CONTROL_H
#define LINE_PID_CONTROL_H

#include "pid.h"
#include "pose2D.h"
#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <turtlesim/Pose.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

namespace navigation
{

    enum Shift
    {
        NORMAL = 0,

        POSITION_LEFT = 1,  // 车往左偏，需要一个向右的角速度
        POSITION_RIGHT = 2, //车往右偏，需要一个向左的角速度
        HEAGING_LEFT , // 位置正确，车头方向正确
        HEADING_RIGHT, // 
    };

    class LinePIDControl
    {
    public:
        LinePIDControl() = default;
        LinePIDControl(ros::NodeHandle *nh);

        ~LinePIDControl(){};
        // ros param get and set to pidcontrol
        void Init();

        void PoseCallBack(const nav_msgs::Odometry &msg);
        void TurtulPoseCallBack(const turtlesim::Pose &msg);
        void GpsCallBack(const nav_msgs::Path &msg);
        void StartCallBack(const std_msgs::Bool &msg);
        // compute the twist msg output through pid
        void OutputCompute();

        // judge current vehicle state (normal or left shift or right shift)
        Shift VehicleState();

        // constrain delta theta
        void NormalizeTheta(double heading, double &theta);

        void Run();

    private:
        ros::NodeHandle *nh_;
        PIDControl vel_pid_;
        PIDControl omega_pid_;
        ros::Publisher twist_pub_;
        ros::Subscriber slam_pose_sub_;
        ros::Subscriber gps_pose_sub_;
        ros::Subscriber turtle_pose_sub_;

        ros::Subscriber pid_start_signal_;

        Pose2D start_pose_;
        Pose2D cur_pose_;
        double heading_;
        double target_vel_;
        double cur_vel_;
        geometry_msgs::Twist cmd_vel_msg_;
        bool enable_position_control_;
        double enable_position_distance_;
        bool enable_slam_;
        bool enable_gps_;
        bool enable_turtle_;
        double heading_tolerant_;
        bool pid_begin_;
    };

} // namespace navigation

#endif // LINE_PID_CONTROL_H