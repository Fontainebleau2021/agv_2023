#include "servo_motor_control.h"

ServoMotorControl::ServoMotorControl() : ros_modbus_()
{
    // ROS_INFO("into construct");
    ros::NodeHandle motor_nh("~servo_motor");
    motor_nh.param("wheel_base", robot_param_.wheel_base, 0.775);
    motor_nh.param("wheel_radius", robot_param_.wheel_radius, 0.10);
    motor_nh.param("transfer", robot_param_.transfer, 60.0);
    motor_nh.param("max_wheel_speed", robot_param_.max_wheel_speed, 0.5);
    motor_nh.param("max_omega", robot_param_.max_omega, 0.4);
    motor_nh.param("left_slave_addr", left_slave_addr_, 2);
    motor_nh.param("right_slave_addr", right_slave_addr_, 1);
    // 轮速和电机转速rpm的关系 速度为1m/s
    motor_rotate_coeff_ = 1 / robot_param_.wheel_radius / 2 / 3.1415926 * robot_param_.transfer * 60;
    // printf("%f \n", robot_param_.wheel_base);
    std::string cmd_vel_topic, remote_cmd_topic;
    // ros_modbus_ = RosModbus();
    motor_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");
    motor_nh.param<std::string>("remote_cmd_topic", remote_cmd_topic, "/remote");
    robot_vel_sub_ = motor_nh.subscribe(cmd_vel_topic, 1, &ServoMotorControl::RobotVelCallBack, this);
    remote_cmd_sub_ = motor_nh.subscribe(remote_cmd_topic, 1, &ServoMotorControl::RemoteCallBack, this);
    //
    v_final_ = 0;
    w_final_ = 0;
    is_remote_ = false;
}

void ServoMotorControl::RobotVelCallBack(const geometry_msgs::Twist &cmd_vel)
{
    v_final_ = cmd_vel.linear.x;
    w_final_ = cmd_vel.angular.z;
}

void ServoMotorControl::ConfigServoMotor()
{
    // 配置伺服控制模式 P001
    ros_modbus_.WriteSingleRegister(right_slave_addr_, 1, 11);
    ros_modbus_.WriteSingleRegister(left_slave_addr_, 1, 11);

    // P005通信指令选择 位置寄存器地址 290 +k  速度：324+k 力矩：358+k  速度寄存器地址默认为324
    ros_modbus_.WriteSingleRegister(right_slave_addr_, 5, 0);
    ros_modbus_.WriteSingleRegister(left_slave_addr_, 5, 0);
    // 485通信波特率设置 P010
    ros_modbus_.WriteSingleRegister(right_slave_addr_, 10, 1);
    ros_modbus_.WriteSingleRegister(left_slave_addr_, 10, 1);

    // 惯量比 负载惯量与电机转子惯量比值的100倍 P032

    // PID增益选择 P060 P061

    // P201 伺服控制模式读取 202 报警状态读取

    // 182电机代码

    // p221反馈速度

    // 235 电机不转原因
}

void ServoMotorControl::RobotVelocityConstrain(double &right_wheel_speed, double &left_wheel_speed)
{
    double scale = 1;
    if (abs(right_wheel_speed) > robot_param_.max_wheel_speed || abs(left_wheel_speed) > robot_param_.max_wheel_speed) // 限制车轮最大速度
    {
        double base = abs(left_wheel_speed);
        if (abs(right_wheel_speed) > abs(left_wheel_speed))
        {
            base = abs(right_wheel_speed);
        }
        scale = robot_param_.max_wheel_speed / base;
        right_wheel_speed = scale * right_wheel_speed;
        left_wheel_speed = scale * left_wheel_speed;
    }
    ROS_INFO("wheel  speed  rightwheel: %f leftwheel: %f  ", right_wheel_speed, left_wheel_speed);
    return;
}

void ServoMotorControl::SendVW(double v, double w)
{
    double scale = 1;
    if (abs(w) > robot_param_.max_omega)
    {
        scale = robot_param_.max_omega / abs(w);
        v = scale * v;
        w = scale * w;
    }
    double right_wheel_speed = v + 0.5 * w * robot_param_.wheel_base;
    double left_wheel_speed = v - 0.5 * w * robot_param_.wheel_base;
    RobotVelocityConstrain(right_wheel_speed, left_wheel_speed);
    // 发送速度在324 范围-6000～6000 rpm单位  播种机电机转速<3000
    short left_motor_data = left_wheel_speed * motor_rotate_coeff_;
    short right_motor_data = right_wheel_speed * motor_rotate_coeff_;
    ROS_INFO("motor rotate speed. right_motor: %d rpm left_motor: %d rpm ", right_motor_data, left_motor_data);
    // 速度寄存器地址默认为324
    ros_modbus_.WriteSingleRegister(right_slave_addr_, 324, right_motor_data);
    ros_modbus_.WriteSingleRegister(left_slave_addr_, 324, left_motor_data);
}

void ServoMotorControl::StopReason()
{
    short right_data = ros_modbus_.ReadRegister(right_slave_addr_, 235, 1);
    ROS_INFO("right wheel stop reason %d", right_data);
    short left_data = ros_modbus_.ReadRegister(left_slave_addr_, 235, 1);
    ROS_INFO("left wheel stop reason %d", left_data);
}

void ServoMotorControl::RobotControl()
{
    if (is_remote_)
    {
        v_final_ = 0;
        w_final_ = 0;
        return;
    }
    else
    {
        SendVW(v_final_, w_final_);
        return;
    }
}

void ServoMotorControl::MotorRPMRead(short &right, short &left)
{
    right = ros_modbus_.ReadRegister(right_slave_addr_, 221, 1);
    left = ros_modbus_.ReadRegister(left_slave_addr_, 221, 1);
    ROS_INFO("Feedback motorRPM. right_motor: %d rpm left_motor: %d rpm ", right, left);
}
