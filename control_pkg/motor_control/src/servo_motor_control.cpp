#include "servo_motor_control.h"

ServoMotorControl::ServoMotorControl() : ros_modbus_()
{
    // ROS_INFO("into construct");
    ros::NodeHandle motor_nh("~/servo_motor");
    motor_nh.param("wheel_base", robot_param_.wheel_base, 0.775);
    motor_nh.param("wheel_radius", robot_param_.wheel_radius, 0.10);
    motor_nh.param("transfer", robot_param_.transfer, 60.0);
    motor_nh.param("max_wheel_speed", robot_param_.max_wheel_speed, 0.5);
    motor_nh.param("max_omega", robot_param_.max_omega, 0.4);
    motor_nh.param("left_slave_addr", left_slave_addr_, 1);
    motor_nh.param("right_slave_addr", right_slave_addr_, 2);
    ROS_INFO("%f",robot_param_.wheel_base);
    // 轮速和电机转速rpm的关系 速度为1m/s
    motor_rotate_coeff_ = -1 / robot_param_.wheel_radius / 2 / 3.1415926 * robot_param_.transfer * 60;
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

    //电机使能
//ros_modbus_.WriteSingleRegister(right_slave_addr_, 2, 0xc4);
  // ros_modbus_.WriteSingleRegister(left_slave_addr_, 2, 0xc4);
    ros_modbus_.WriteSingleRegister(left_slave_addr_, 0, 0x01);
    ros_modbus_.WriteSingleRegister(right_slave_addr_, 0, 0x01);
}

void ServoMotorControl::RobotVelCallBack(const geometry_msgs::Twist &cmd_vel)
{
    v_final_ = cmd_vel.linear.x;
    w_final_ = cmd_vel.angular.z;
}

void ServoMotorControl::ConfigServoMotor()
{
    // 速度模式 地址02 数据0xc4
    ros_modbus_.WriteSingleRegister(right_slave_addr_, 2, 0xc4);
    ros_modbus_.WriteSingleRegister(left_slave_addr_, 2, 0xc4);

    //速度模式下必要的寄存器操作
    // 地址00  发送数据0 停机   发送数据1 启动
    ros_modbus_.WriteSingleRegister(left_slave_addr_, 0, 0x00);
    ros_modbus_.WriteSingleRegister(left_slave_addr_, 0, 0x01);
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
    //限制一下最大的旋转角速度
    double scale = 1;
    if (abs(w) > robot_param_.max_omega)
    {
        scale = robot_param_.max_omega / abs(w);
        v = scale * v;
        w = scale * w;
    }
    //再限制车轮最大速度
    double right_wheel_speed = v + 0.5 * w * robot_param_.wheel_base;
    double left_wheel_speed = v - 0.5 * w * robot_param_.wheel_base;
    RobotVelocityConstrain(right_wheel_speed, left_wheel_speed);
    // 发送速度在324 范围-6000～6000 rpm单位  播种机电机转速<3000
    short left_motor_rotate = left_wheel_speed * motor_rotate_coeff_;
    short right_motor_rotate = right_wheel_speed * motor_rotate_coeff_;
    ROS_INFO("motor rotate speed. right_motor: %d rpm left_motor: %d rpm ", right_motor_rotate, left_motor_rotate);
    // 速度寄存器地址默认为6  转速= 写入值 /8192 * 3000  写入值 = 转速 /3000 *8192
    short left_motor_data = left_wheel_speed * motor_rotate_coeff_ * 8192 / 3000;
    short right_motor_data = right_wheel_speed * motor_rotate_coeff_ * 8192 / 3000;
    ROS_INFO("Write register data (MAX: 8192 input =  /3000 *8192). right_motor: %d rpm left_motor: %d rpm ", right_motor_data, left_motor_data);
    //short data = 100;
ros_modbus_.WriteSingleRegister(right_slave_addr_, 6, right_motor_data);
    ros_modbus_.WriteSingleRegister(left_slave_addr_, 6, left_motor_data);
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
	//short right_data = ros_modbus_.ReadRegister(right_slave_addr_, 0xE1, 1);

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
