#include "PID_control.h"
#include <tf/transform_datatypes.h>

namespace navigation
{
    LinePIDControl::LinePIDControl(ros::NodeHandle *nh) : nh_(nh), pid_begin_(false)
    {
        Init();
    }

    void LinePIDControl::Init()
    {
        double v_kp, v_ki, v_kd, o_kp, o_ki, o_kd, sample_time;
        double v_inter_max, v_inter_min, o_inter_max, o_inter_min;
        double v_output_max, v_output_min, o_output_max, o_output_min;
        double v_tau, o_tau;
        nh_->param("target_vel", target_vel_, 1.0);
        nh_->param("sample_time", sample_time, 0.2);

        nh_->param("vel_pid_kp", v_kp, 0.8);
        nh_->param("vel_pid_ki", v_ki, 0.8);
        nh_->param("vel_pid_kd", v_kd, 0.8);
        nh_->param("vel_inter_max", v_inter_max, 0.8);
        nh_->param("vel_inter_min", v_inter_min, 0.8);
        nh_->param("vel_output_max", v_output_max, 0.8);
        nh_->param("vel_output_min", v_output_min, 0.8);
        nh_->param("vel_tau", v_tau, 0.01);

        nh_->param("omega_pid_kp", o_kp, 0.8);
        nh_->param("omega_pid_ki", o_ki, 0.8);
        nh_->param("omega_pid_kd", o_kd, 0.8);
        nh_->param("omega_inter_max", o_inter_max, 0.8);
        nh_->param("omega_inter_min", o_inter_min, 0.8);
        nh_->param("omega_output_max", o_output_max, 0.8);
        nh_->param("omega_output_min", o_output_min, 0.8);
        nh_->param("omega_tau", o_tau, 0.01);

        vel_pid_ = PIDControl(v_kp, v_ki, v_kd, sample_time, v_output_max, v_output_min, v_inter_max, v_inter_min, v_tau);
        omega_pid_ = PIDControl(o_kp, o_ki, o_kd, sample_time, o_output_max, o_output_min, o_inter_max, o_inter_min, o_tau);

        nh_->param("line_heading", heading_, 0.0);

        std::string twist_topic, slam_pose_sub_topic, gps_pose_sub_topic, turtle_pose_sub_topic;
        nh_->param<std::string>("twist_topic", twist_topic, "/cmd_vel");
        nh_->param<std::string>("slam_pose_sub_topic", slam_pose_sub_topic, "/odometry");
        nh_->param<std::string>("gps_pose_sub_topic", gps_pose_sub_topic, "/fused_path");
        nh_->param<std::string>("turtle_pose_sub_topic", turtle_pose_sub_topic, "turtle1/pose");
        twist_pub_ = nh_->advertise<geometry_msgs::Twist>(twist_topic, 10);

        nh_->param("enable_slam", enable_slam_, false);
        nh_->param("enable_gps", enable_gps_, false);
        nh_->param("enable_turtle", enable_turtle_, false);
        nh_->param("enable_position_control", enable_position_control_, false);
        nh_->param("enable_position_distance", enable_position_distance_, 10.0);
        nh_->param("heading_tolerant", heading_tolerant_, 0.02);
        pid_start_signal_ = nh_->subscribe("/pid_start", 1, &LinePIDControl::StartCallBack, this);
        if (enable_turtle_)
            turtle_pose_sub_ = nh_->subscribe(turtle_pose_sub_topic, 1, &LinePIDControl::TurtulPoseCallBack, this);
        if (enable_slam_)
            slam_pose_sub_ = nh_->subscribe(slam_pose_sub_topic, 1, &LinePIDControl::PoseCallBack, this);
        if (enable_gps_)
            gps_pose_sub_ = nh_->subscribe(gps_pose_sub_topic, 1, &LinePIDControl::GpsCallBack, this);
    }

    void LinePIDControl::StartCallBack(const std_msgs::Bool &msg)
    {
        pid_begin_ = msg.data;
    }

    void LinePIDControl::PoseCallBack(const nav_msgs::Odometry &msg)
    {
        cur_vel_ = msg.twist.twist.linear.x;
        tf::Quaternion quat;
        if (abs(cur_vel_) < 0.01 && !pid_begin_)
        {
            start_pose_.x = msg.pose.pose.position.x;
            start_pose_.y = msg.pose.pose.position.y;

            tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
            double roll, pitch, yaw;                      //定义存储r\p\y的容器
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换
            start_pose_.theta = yaw;
            ROS_INFO("vehicle static position x: %f, y: %f, theta: %f", start_pose_.x, start_pose_.y, start_pose_.theta);
            return;
        }
        cur_pose_.x = msg.pose.pose.position.x;
        cur_pose_.y = msg.pose.pose.position.y;
        tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
        double roll, pitch, yaw;                      //定义存储r\p\y的容器
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换
        cur_pose_.theta = yaw;
        ROS_INFO("slam position x: %f, y: %f, theta: %f", cur_pose_.x, cur_pose_.y, cur_pose_.theta);
        // NormalizeTheta(headingcur_pose_.theta);
    }

    void LinePIDControl::GpsCallBack(const nav_msgs::Path &msg)
    {
        auto latest_pose = msg.poses.back();
        cur_pose_.x = latest_pose.pose.position.x;
        cur_pose_.y = latest_pose.pose.position.y;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(latest_pose.pose.orientation, quat);
        double roll, pitch, yaw;                      //定义存储r\p\y的容器
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换
        cur_pose_.theta = yaw;
        ROS_INFO("gps position x: %f, y: %f, theta: %f", cur_pose_.x, cur_pose_.y, cur_pose_.theta);
        // NormalizeTheta(cur_pose_.theta);
        //  geometry_msgs::Pose latest = msg.
    }

    void LinePIDControl::TurtulPoseCallBack(const turtlesim::Pose &msg)
    {
        cur_vel_ = msg.linear_velocity;
        if (abs(cur_vel_) < 0.01 && !pid_begin_)
        {
            start_pose_.x = msg.x;
            start_pose_.y = msg.y;
            start_pose_.theta = msg.theta;
            ROS_INFO("turtle static position x: %f, y: %f, theta: %f", start_pose_.x, start_pose_.y, start_pose_.theta);
            return;
        }
        cur_pose_.x = msg.x;
        cur_pose_.y = msg.y;
        cur_pose_.theta = msg.theta;
        // NormalizeTheta(cur_pose_.theta);
        ROS_INFO("turtle position x: %f, y: %f, theta: %f", cur_pose_.x, cur_pose_.y, cur_pose_.theta);
    }

    Shift LinePIDControl::VehicleState()
    {
        // if (abs(cur_vel_) < 0.01)
        // {
        //     NormalizeTheta(heading_, start_pose_.theta);
        //     if (abs(start_pose_.theta - heading_) > heading_tolerant_)
        //     {
        //         return Shift::NORMAL;
        //     }
        //     else if ((start_pose_.theta - heading_) > heading_tolerant_)
        //     {
        //         return Shift::HEAGING_LEFT;
        //     }
        //     else
        //     {
        //         return Shift::HEADING_RIGHT;
        //     }
        // }
        
        double delta_x = cur_pose_.x - start_pose_.x;
        double delta_y = cur_pose_.y - start_pose_.y;
        double dis = hypot(delta_x, delta_y);
        ROS_INFO("move ids %f", dis);
        if(hypot(delta_x,delta_y)<enable_position_distance_)
        {
            ROS_INFO("NOW is just velcity and theta control");
            return Shift::NORMAL;
        }
        double slope = atan2(delta_y, delta_x);
        ROS_INFO("slope from cur pose to start pose: %f", slope);
        if (abs(slope - heading_) < heading_tolerant_)
        {
            return Shift::NORMAL;
        }
        else if ((slope - heading_) > heading_tolerant_)
        {
            return Shift::POSITION_LEFT;
        }
        else
        {
            return Shift::POSITION_RIGHT;
        }
    }

    void LinePIDControl::OutputCompute()
    {
        double cur_heading = heading_;
        if (enable_position_control_)
        {
            Shift vehicle_shift = VehicleState();
            switch (vehicle_shift)
            {
            case Shift::NORMAL:
                ROS_INFO("vehicle position is normal");
                break;
            case Shift::POSITION_RIGHT:
                ROS_INFO("vehicle is shift to right");
                cur_heading = heading_ + 0.08;
                break;
            case Shift::POSITION_LEFT:
                ROS_INFO("vehicle is shift to left");
                cur_heading = heading_ - 0.08;
                break;
            default:
                break;
            }
        }
        NormalizeTheta(cur_heading, cur_pose_.theta);
        double omega = omega_pid_.CalculateOutput(cur_heading, cur_pose_.theta);
        double vel = vel_pid_.CalculateOutput(target_vel_, cur_vel_);
        cmd_vel_msg_.linear.x = vel;
        cmd_vel_msg_.angular.z = omega;
        ROS_INFO("vel: %f  omega: %f", vel, omega);
        twist_pub_.publish(cmd_vel_msg_);
    }

    void LinePIDControl::NormalizeTheta(double heading, double &theta)
    {
        // while (theta >= 2 *M_PI)
        // {
        //     /* code */
        //     theta -= 2 * M_PI;
        // }
        // while (theta < 0)
        // {
        //     theta += 2 * M_PI;
        // }
        if ((heading - theta) > M_PI)
        {
            theta += 2 * M_PI;
        }
        else if ((heading - theta) < -M_PI)
        {
            theta -= 2 * M_PI;
        }
    }

    void LinePIDControl::Run()
    {
        if (pid_begin_)
        {

            OutputCompute();
        }
        else
        {
            ROS_INFO("pid control has not been activated");
        }
    }

} // namespace navigation

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_control");
    ros::NodeHandle nh("~"); //私有名称以节点的名字作为命名空间
    double rate;
    nh.param("rate", rate, 5.0);
    navigation::LinePIDControl pid_control(&nh);
    ros::Rate loop_rate(rate);
    // navi_master_ros.ToGridCellsMsg();
    while (ros::ok())
    {
        ROS_INFO("in loop");
        pid_control.Run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}