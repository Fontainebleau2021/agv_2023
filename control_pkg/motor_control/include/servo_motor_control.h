#ifndef SERVO_MOTOR_CONTROL_H
#define SERVO_MOTOR_CONTROL_H

#include "ros_modbus.h"

#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>


/*
功能：订阅发布的速度指令 求解电机转速发送给电机驱动器
*/
 


struct RobotParam
{
    double wheel_base;
    double wheel_radius;
    double transfer;
    double max_wheel_speed;
    double max_omega;
};



class ServoMotorControl
{
public:

    ServoMotorControl();
    ~ServoMotorControl(){}
    //配置电机参数 基本上用不到
    void ConfigServoMotor();
    //回调函数
    void RobotVelCallBack(const geometry_msgs::Twist &cmd_vel);
    void RemoteCallBack(const std_msgs::Bool &remote_signal)
    {
        is_remote_ = remote_signal.data;
    }
    //发送速度指令给驱动器
    void SendVW(double v, double w);
    
    //机器人速度约束 归一化计算  左右两轮的速度都无法超过最大速度
    void RobotVelocityConstrain(double& right_wheel_speed, double& left_wheel_speed);
    // 读取驱动器状态寄存器
    void StopReason();
    //主运行函数
    void RobotControl();
private:
    /* data */
    RosModbus ros_modbus_;
    double v_final_;
    double w_final_;
    RobotParam robot_param_;
    double motor_rotate_coeff_;
    int left_slave_addr_;
    int right_slave_addr_;
    bool is_remote_;

    ros::Subscriber robot_vel_sub_;
    ros::Subscriber remote_cmd_sub_;
};







#endif // SERVO_MOTOR_CONTROL_H