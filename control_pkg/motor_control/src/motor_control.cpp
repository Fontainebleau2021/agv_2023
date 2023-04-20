#include "motor_control.h"

int MotorControl::Init()
{
    try
    {
        //设置串口属性，并打开串口
        ser_.setPort(port_);
        ser_.setParity(serial::parity_none);
        ser_.setBaudrate(baudrate_);
        ser_.setStopbits(serial::stopbits_one);
        serial::Timeout to = serial::Timeout::simpleTimeout(timeout_);
        ser_.setTimeout(to);
        ser_.open();
    }
    catch (serial::IOException &e)
    {
        ROS_INFO(e.what());
        ROS_ERROR("Unable to open port ");
        return -1;
    }

    //检测串口是否已经打开，并给出提示信息
    if (ser_.isOpen())
    {
        ROS_INFO("Serial Port initialized");
        return 1;
    }
    else
    {
        return -1;
    }
}
void MotorControl::PWMInRange(double &left_pwm, double &right_pwm)
{
    //if (abs(left_pwm) <200 )
}
void MotorControl::SendLeftRight(double left_vel, double right_vel)
{
    double pwm_coeff = 990 / maxSpeed;
    short velRight = (right_vel)*pwm_coeff;
    short velLeft = (left_vel)*pwm_coeff;
    ROS_INFO("wheel  speed  rightwheel: %f leftwheel: %f  ", right_vel, left_vel);
    ROS_INFO("motor driver frequency input rightwheel: %d leftwheel: %d  ", velRight, velLeft);
    unsigned char uchCRCHi = 0xFF, uchCRCLo = 0xFF;
    unsigned char data1[8] = {0x02, 0x06, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00}; // 06 represent writing single register, 03 is the address of right controler
    data1[4] = (unsigned char)(velRight >> 8);
    data1[5] = (unsigned char)(velRight & 0xff);
    CRC16(data1, 6, uchCRCHi, uchCRCLo);
    data1[7] = uchCRCHi;
    data1[6] = uchCRCLo;

    uchCRCHi = 0xFF;
    uchCRCLo = 0xFF;
    unsigned char data2[8] = {0x01, 0x06, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00}; // 43 is the aim speed to be controled
    data2[4] = (unsigned char)(velLeft >> 8);
    data2[5] = (unsigned char)(velLeft & 0xff);
    CRC16(data2, 6, uchCRCHi, uchCRCLo);
    data2[7] = uchCRCHi;
    data2[6] = uchCRCLo;
    ser_.write(data1, 8);
    ser_.waitReadable(); // 严重延迟
    ROS_INFO("available\t%d", ser_.available());
    unsigned char receive_data[8];
    int data_len = ser_.available();
    if (ser_.available())
        ser_.read(receive_data, 8);
    // ROS_INFO("RIGHT Response from driver %sx", receive_data);
    for (int i = 0; i < data_len; i++)
    {
        if (receive_data[i] != data1[i])
        {
            ROS_WARN("motor driver receive data is false");
        }
    }
    ser_.write(data2, 8);
    ser_.waitReadable(); // 严重延迟
    ROS_INFO("available\t%d", ser_.available());
    if (ser_.available())
        ser_.read(receive_data, 8);
    for (int i = 0; i < data_len; i++)
    {
        if (receive_data[i] != data1[i])
        {
            ROS_WARN("motor driver receive data is false");
        }
    }
}

void MotorControl::SendVW(double v, double w)
{
    double scale = 1;
    if (abs(w) > maxOmega)
    {
        scale = maxOmega / abs(w);
        v = scale * v;
        w = scale * w;
    }
    double right_wheel_vel = v + 0.5 * w * WHEEL_BASE;
    double left_wheel_vel = v - 0.5 * w * WHEEL_BASE;
    if (abs(right_wheel_vel) > maxSpeed || abs(left_wheel_vel) > maxSpeed) //限制车轮最大速度为
    {
        short base = abs(left_wheel_vel);
        if (abs(right_wheel_vel) > abs(left_wheel_vel))
        {
            base = abs(right_wheel_vel);
        }
        scale = maxSpeed / base;
        right_wheel_vel = scale * right_wheel_vel;
        left_wheel_vel = scale * left_wheel_vel;
    }
    ROS_INFO("final speed  V: %f w: %f  ", v, w);

    SendLeftRight(left_wheel_vel, right_wheel_vel);
}

void MotorControl::WriteRelay(const unsigned char addr, const unsigned char w_data)
{
    unsigned char uchCRCHi = 0xFF, uchCRCLo = 0xFF;
    unsigned char data[8] = {0x04, 0x05, 0x00, addr, w_data, 0x00, 0x00, 0x00};
    CRC16(data, 6, uchCRCHi, uchCRCLo);
    data[7] = uchCRCHi;
    data[6] = uchCRCLo;
    printf("CRCHIGH: %x, CRCLOW: %x \n", uchCRCHi, uchCRCLo);
    ser_.write(data, 8);
    ser_.waitReadable();
    unsigned char receivedata[40];
    int data_len = ser_.available();
    ROS_INFO("available\t%d", data_len);
    if (ser_.available())
        ser_.read(receivedata, ser_.available());
    for (int i = 0; i < data_len; i++)
    {
        if (receivedata[i] != data[i])
        {
            ROS_WARN("receive data is false");
        }
    }
}

void MotorControl::RemoteBegin()
{
    WriteRelay(0x01, 0x00);
}

void MotorControl::RemoteCancel()
{
    WriteRelay(0x01, 0xFF);
}

// DO3
void MotorControl::SeedingUp(double up_time)
{
    WriteRelay(0x02, 0xFF);
    ros::Duration(up_time).sleep();
    WriteRelay(0x02, 0x00);
}

// DO4
void MotorControl::SeedingDown(double down_time)
{

    WriteRelay(0x03, 0xFF);
    ros::Duration(down_time).sleep();
    WriteRelay(0x03, 0x00);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_control");
    ros::NodeHandle nh("~");
    int baudrate, timeout, control_rate;

    nh.param("baudrate", baudrate, 9600);
    nh.param("timeout", timeout, 1000);
    nh.param("control_rate", control_rate, 5);
    std::string port_name;
    nh.param<std::string>("port_name", port_name, "/dev/ttyUSB0");
    MotorControl motor(&nh, port_name, baudrate, timeout);
    ros::Rate loop_rate(control_rate);
    while (ros::ok())
    {
        if (motor.Remote())
        {
            motor.RemoteBegin();
            ROS_INFO("Now is in Remote Control");
        }
        else if (!motor.Stop())
        {
            motor.RemoteCancel();
            if (motor.receive_cmd_)
            {
                ROS_INFO("we have received speed cmd");
                motor.SendVW(motor.Vel(), motor.Omega());
                motor.receive_cmd_ = false;
            }
            else
            {
                ROS_INFO("we have not received speed cmd");
                motor.SendVW(0, 0);
            }
        }
        else
        {
            motor.SendVW(0, 0);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}