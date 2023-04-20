
#include "ros_modbus.h"

void CRC16(unsigned char *puchMsg, unsigned short usDataLen, unsigned char &uchCRCHi, unsigned char &uchCRCLo)
{
    unsigned uIndex;    /* CRC ��ѯ������*/
    while (usDataLen--) /* ����������Ļ�����*/
    {
        uIndex = uchCRCHi ^ *puchMsg++; /* ����CRC */
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];
    }
}



int RosModbus::Init()
{
  
    ros::NodeHandle mod_nh("~/modbus");
    mod_nh.param("baudrate", baudrate_, 19200);
    mod_nh.param("timeout", timeout_, 100);
    mod_nh.param<std::string>("port_name", port_, "/dev/ttyUSB0");
    try
    {
        // 设置串口属性，并打开串口
        serial_.setPort(port_);
        serial_.setParity(serial::parity_none);
        serial_.setBaudrate(baudrate_);
        serial_.setStopbits(serial::stopbits_one);
        serial::Timeout to = serial::Timeout::simpleTimeout(timeout_);
        serial_.setTimeout(to);
        serial_.open();
    }
    catch (serial::IOException &e)
    {
        ROS_INFO(e.what());
        ROS_ERROR("Unable to open port ");
        return -1;
    }

    // 检测串口是否已经打开，并给出提示信息
    if (serial_.isOpen())
    {
        ROS_INFO("Serial Port initialized");
        return 1;
    }
    else
    {
        return -1;
    }
}


void RosModbus::PrintData(unsigned char *serial_msg, int len)
{
    while(len--)
    {
        printf("%x, ", *serial_msg++);
    }
    printf("\n");
}

short RosModbus::ReadRegister(unsigned char slave_addr, short register_addr, unsigned char register_num)
{
    unsigned char send[8];
    send[0] = slave_addr;
    send[1] = 0x03;
    send[2] = (unsigned char)(register_addr >> 8);
    send[3] = (unsigned char)(register_addr & 0xFF);
    send[4] = 0;
    send[5] = register_num;
    unsigned char uchCRCHi = 0xFF, uchCRCLo = 0xFF;
    CRC16(send, 6, uchCRCHi, uchCRCLo);
    // 这个需要测试一下
    send[6] = uchCRCHi;
    send[7] = uchCRCLo;
#if test
    PrintData(send, 8);
#else
	serial_.write(send, 8);
    serial_.waitReadable(); // 严重延迟

    unsigned char receive_data[256];
    size_t n = serial_.available();
    if (n == 0)
    {
        ROS_INFO("Do not receive data");
    }
    serial_.read(receive_data, n);
    if (receive_data[0] != slave_addr || receive_data[1] != 0x03)
    {
        ROS_INFO("slave addr or funciton code error");
    }
    CRC16(receive_data, 3 + receive_data[2], uchCRCHi, uchCRCLo);
    if (uchCRCHi != receive_data[3 + receive_data[2]] || uchCRCLo != receive_data[4 + receive_data[2]])
    {
        ROS_INFO("CRC calculate error");
    }
    short data = 0;
    data = data | receive_data[3];
    data = data << 8;
    data = data | receive_data[4];
    return data;	
#endif //test
    
}

void RosModbus::WriteSingleRegister(unsigned char slave_addr, short register_addr, short data)
{
    unsigned char send[8];
    send[0] = slave_addr;
    send[1] = 0x06;
    send[2] = (unsigned char)(register_addr >> 8);
    send[3] = (unsigned char)(register_addr & 0xFF);
    send[4] = (unsigned char)(data >> 8);
    send[5] = (unsigned char)(data & 0xFF);
    unsigned char uchCRCHi = 0xFF, uchCRCLo = 0xFF;
    CRC16(send, 6, uchCRCHi, uchCRCLo);
    // 这个需要测试一下
    send[6] = uchCRCHi;
    send[7] = uchCRCLo;
#if test
    PrintData(send, 8);
#else
    serial_.write(send, 8);
    serial_.waitReadable();
    if(serial_.available()) serial_.read(serial_.available());
#endif //test
}


void RosModbus::WriteMultiRegister(unsigned char slave_addr,short register_addr,short register_num, short* data)
{
    int len = 9+2*register_num;
    unsigned char uchCRCHi = 0xFF, uchCRCLo = 0xFF;
    unsigned char send[len];
    send[0] = slave_addr;
    send[1] = 0x10;
    send[2] = (unsigned char)(register_addr >> 8);
    send[3] = (unsigned char)(register_addr & 0xFF);
    send[4] = (unsigned char)(register_num >> 8);
    send[5] = (unsigned char)(register_num & 0xFF);
    send[6] = 2*send[5];
    for(int i =0;i <register_num; i++)
    {
        send[7+i*2] = (unsigned char)(data[i] >> 8);
        send[7+i*2+1] = (unsigned char)(data[i] & 0xFF);
    }
    CRC16(send, len-2, uchCRCHi, uchCRCLo);
    send[len-2] = uchCRCHi;
    send[len-1] = uchCRCLo;
#if test
    PrintData(send, len);
#else
    serial_.write(send, len);
    serial_.waitReadable();
    if(serial_.available()) serial_.read(serial_.available());
#endif //test
}




