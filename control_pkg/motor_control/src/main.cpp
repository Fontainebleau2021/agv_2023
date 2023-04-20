#include "servo_motor_control.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "servo_motor_control");
    ros::NodeHandle motor_control("~");
    ServoMotorControl motor;
    ros::Rate loop_rate(5);
    //motor.SendVW(0.4, 0.2);
    short left = 0, right = 0;
    while (ros::ok())
    {
        motor.RobotControl();
        motor.MotorRPMRead(right, left);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
