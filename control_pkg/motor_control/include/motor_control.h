#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "CRC.h"

#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include "motor_control/seedingcontrol.h"

#define WHEEL_BASE 0.775                                         //驱动轮间距的一半
#define WHEEL_RADIUS 0.20                                        // 轮子半径
#define TRANSFOR 20                                              //传动比
#define K_coeff 1 / WHEEL_RADIUS * 20 * 60 / 3.1415926 * 10 / 60 // r/min  1m/s速度对应的转速（转每分）HZ 频率  1s转的圈速

#define maxSpeed 0.7 // 单个轮子的最大轮速

#define maxOmega 0.8 // 旋转的最大角速度

#define MIN_FREQ_INPUT 200

//转速和换向频率的关系  n = 20*f/num(极个数)
//电机参数：最大转速1500r/min
// 给定线速度v ——>计算换向频率f  v*(1/WHEEL_RADIUS*60/3.1415926*20*num/20);

class MotorControl
{
public:
    MotorControl(ros::NodeHandle *nh, std::string name, int baudrate, int timeout)
        : nh_(nh), port_(name), baudrate_(baudrate), timeout_(timeout)
    {
        v_final_ = 0;
        w_final_ = 0;
        Init();
        RemoteBegin();
        vel_sub_ = nh_->subscribe("/cmd_vel", 10, &MotorControl::VelCallBack, this);
        stop_sub_ = nh_->subscribe("/stop", 10, &MotorControl::StopCallBack, this);
        remote_sub_ = nh_->subscribe("/remote", 10, &MotorControl::RemoteCallBack, this);
        seeding_sub_ = nh_->subscribe("/seeding", 1, &MotorControl::SeedingControlCallBack, this);
    }
    ~MotorControl() {}
    int Init();
    void VelCallBack(const geometry_msgs::Twist &cmd_vel)
    {
        v_final_ = cmd_vel.linear.x;
        w_final_ = cmd_vel.angular.z;
        receive_cmd_ = true;
    }

    void StopCallBack(const std_msgs::Bool &stop_signal)
    {
        stop_ = stop_signal.data;
    }

    void RemoteCallBack(const std_msgs::Bool &remote_signal)
    {
        remote_control_ = remote_signal.data;
    }
    void SeedingControlCallBack(const motor_control::seedingcontrol &msg)
    {
        if (msg.dir == 0)
        {
            SeedingUp(msg.time);
        }
        else
        {
            SeedingDown(msg.time);
        }
    }
    void WriteRelay(const unsigned char addr, const unsigned char data);
    void PWMInRange(double &left_pwm, double &right_pwm);
    void SendLeftRight(double left_vel, double right_vel);
    void SendVW(double v, double w);
    double Vel() { return v_final_; }
    double Omega() { return w_final_; }
    bool Stop() { return stop_; }
    bool Remote() { return remote_control_; }
    void RemoteCancel();
    void RemoteBegin();
    void SeedingUp(double up_time);
    void SeedingDown(double down_time);

    bool receive_cmd_ = false;

private:
    ros::NodeHandle *nh_;
    serial::Serial ser_;
    std::string port_;
    int baudrate_;
    int timeout_;
    double v_final_;
    double w_final_;
    ros::Subscriber vel_sub_;
    ros::Subscriber stop_sub_;
    ros::Subscriber remote_sub_;
    ros::Subscriber seeding_sub_;
    bool stop_ = false;
    bool remote_control_ = true;

    /* data */
};

#endif // MOTOR_CONTROL_H