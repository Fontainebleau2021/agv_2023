#include "ros_modbus.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_modbus_test");
    ros::NodeHandle modbus_test("~");
    RosModbus ros_modbus;
    ros::Rate loop_rate(5);
    short slave_addr = 0x01;
    short reg_addr = 0x00e1;
    short data = 0x1000;
    short w_reg_addr = 0x06;
    short m_data[] = {1000, 2000};
    short mw_reg_addr = 325;
    while (ros::ok())
    {
        ros_modbus.ReadRegister(slave_addr,reg_addr,1);
        ros_modbus.WriteSingleRegister(slave_addr,w_reg_addr,data);
        ros_modbus.WriteMultiRegister(slave_addr,mw_reg_addr,2,m_data);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
