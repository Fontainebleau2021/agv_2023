#include <iostream>
#include <string>
#include <sstream>
#include "ros/ros.h"
#include "robotsocket/harvestingRobotClient.h"
#include "robotsocket/harvest_state.h"
#include "robotsocket/harvest_vel.h"

// 左右轮电压解析函数，输入左右轮转速，输出发送的命令
void HarvestingRobotClient::harvest_vel_cmd(double left_vel, double right_vel, unsigned char* cmd){
    cmd[0]=0x3c;
    cmd[1]=0x30;
    cmd[2]=0x35;
    cmd[3]=0x7c;

    char left_vel_cmd[4];
    char right_vel_cmd[4];
    if(left_vel>2000){
        left_vel=2000;
    }else if(left_vel<-2000){
        left_vel=-2000;
    }
    if(right_vel>2000){
        right_vel=2000;
    }else if(right_vel<-2000){
        right_vel=-2000;
    }
    int2hex(left_vel,8,left_vel_cmd);
    int2hex(right_vel,8,right_vel_cmd);
    cmd[4] = left_vel_cmd[0];
    cmd[5] = left_vel_cmd[1];
    cmd[6] = left_vel_cmd[2];
    cmd[7] = left_vel_cmd[3];
    cmd[8] = right_vel_cmd[0];
    cmd[9] = right_vel_cmd[1];
    cmd[10] = right_vel_cmd[2];
    cmd[11] = right_vel_cmd[3];

    cmd[12]=0x3e;
    cmd[13] = '\0';
}

// 收割装置电压解析函数，输入割刀电压、前端输送电机电压，输出发送的命令
void HarvestingRobotClient::harvest_cut_device_cmd(double knife_voltage, double shake_voltage, unsigned char* cmd){
    cmd[0]=0x3c;
    cmd[1]=0x30;
    cmd[2]=0x36;
    cmd[3]=0x7c;
    if(knife_voltage>3){
        knife_voltage=3;
    }else if(knife_voltage<0){
        knife_voltage=0;
    }
    if(shake_voltage>3){
        shake_voltage=3;
    }else if(shake_voltage<0){
        shake_voltage=0;
    }
    int knife_vol=(knife_voltage+10)/20*65535;
    int shake_vol=(shake_voltage+10)/20*65535;
    char knife_res[2];
    char shake_res[2];
    int2hex(knife_vol,4,knife_res);
    int2hex(shake_vol,4,shake_res);
    cmd[4] = knife_res[0];
    cmd[5] = knife_res[1];
    cmd[6] = shake_res[0];
    cmd[7] = shake_res[1];

    cmd[8]=0x00;
    cmd[9]=0x00;
    cmd[10]=0x00;
    cmd[11]=0x00;
    cmd[12]=0x3e;
    cmd[13] = '\0';
}

// 传送带电压解析函数，输入传送带电压，输出发送的命令
void HarvestingRobotClient::harvest_conveyor_cmd(double voltage, unsigned char* cmd){
    cmd[0]=0x3c;
    cmd[1]=0x30;
    cmd[2]=0x37;
    cmd[3]=0x7c;
    if(voltage>3){
        voltage=3;
    }else if(voltage<0){
        voltage=0;
    }
    int vol=(voltage+10)/20*65535;
    char res[2];
    int2hex(vol,4,res);
    cmd[4] = res[0];
    cmd[5] = res[1];

    cmd[6]=0x80;
    cmd[7]=0x00;
    cmd[8]=0x00;
    cmd[9]=0x00;
    cmd[10]=0x00;
    cmd[11]=0x00;
    cmd[12]=0x3e;
    cmd[13] = '\0';
}

// 接收命令，解析命令，发布ROS话题/harvest_state
void HarvestingRobotClient::harvest_pub(unsigned char * cmd, int cmd_length){
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<robotsocket::harvest_state>("/harvest_state", 100, true);
    robotsocket::harvest_state msg;
   
    // 开始符 1字节 0
    
    // 16路DI 2字节 1 2
    char res1[8],res2[8];
    hex2bin(cmd[1],res1,8);
    hex2bin(cmd[2],res2,8);
    for(int i=0;i<8;i++){
        if(res1[i]=='0'){
            msg.DI[i]=0;
        }else{
            msg.DI[i]=1;
        }
    }
    for(int i=0;i<8;i++){
        if(res2[i]=='0'){
            msg.DI[i+8]=0;
        }else{
            msg.DI[i+8]=1;
        }
    }

    // 上下限位开关
    

    // 8路模拟量 16字节 3-18
    int analog_dec1 = hexChar2decInt(cmd+7,2);
    int analog_dec2 = hexChar2decInt(cmd+9,2);
    int analog_dec3 = hexChar2decInt(cmd+11,2);
    int analog_dec4 = hexChar2decInt(cmd+13,2);
    msg.analog[2]=((double)analog_dec1*20/65536-1.06)/0.459*110;
    msg.analog[3]=((double)analog_dec2*20/65536);
    msg.analog[4]=((double)analog_dec3*20/65536);
    msg.analog[5]=((double)analog_dec4*20/65536);

    // 16路DO 2字节 19 20
    char res19[8],res20[8];
    hex2bin(cmd[19],res19,8);
    hex2bin(cmd[20],res20,8);
    for(int i=0;i<8;i++){
        if(res19[i]=='0'){
            msg.DO[i]=0;
        }else{
            msg.DO[i]=1;
        }
    }
    for(int i=0;i<8;i++){
        if(res20[i]=='0'){
            msg.DO[i+8]=0;
        }else{
            msg.DO[i+8]=1;
        }
    }
    
    // 备用 19字节 21-39
    // 陀螺仪数据 6字节 40-45

    //CAN1母线电压 2字节 46 47
    // CAN2母线电压 2字节 48 49
    int can_u_dec1 = hexChar2decInt(cmd+46,-2);
    int can_u_dec2 = hexChar2decInt(cmd+48,-2);
    msg.voltage[0]=(double)can_u_dec1/10;
    msg.voltage[1]=(double)can_u_dec2/10;

    // CAN1实际转速 4字节 50-53
    // CAN2实际转速 4字节 54-57
    int can_vel_dec1 = hexChar2decInt(cmd+50,4);
    int can_vel_dec2 = hexChar2decInt(cmd+54,4);
    msg.vel[0]=(double)can_vel_dec1;
    msg.vel[1]=(double)can_vel_dec2;

    // CAN1电流 2字节 58 59
    // CAN2电流 2字节 60 61
   int can_i_dec1 = hexChar2decInt(cmd+58,2);
    int can_i_dec2 = hexChar2decInt(cmd+60,2);
    msg.current[0]=(double)can_i_dec1/10;
    msg.current[1]=(double)can_i_dec2/10;

    // CAN1散热片温度 2字节 62 63
    // CAN2散热片温度 2字节 64 65
    // CAN1CPU温度 2字节 66 67
    // CAN2CPU温度 2字节 68 69
    int can_temp_dec1 = hexChar2decInt(cmd+62,2);
    int can_temp_dec2 = hexChar2decInt(cmd+64,2);
    int can_temp_dec3 = hexChar2decInt(cmd+66,2);
    int can_temp_dec4 = hexChar2decInt(cmd+68,2);
    if(can_temp_dec1>75||can_temp_dec2>75||can_temp_dec3>75||can_temp_dec4>75){
        harvest_temp_state = 1;
        cout<<"过热！！！"<<endl;
    }
    if(can_temp_dec1<70||can_temp_dec2<70||can_temp_dec3<70||can_temp_dec4<70){
        harvest_temp_state = 0;
    }
    msg.temperature[0]=(double)can_temp_dec1;
    msg.temperature[1]=(double)can_temp_dec2;
    msg.temperature[2]=(double)can_temp_dec3;
    msg.temperature[3]=(double)can_temp_dec4;

    // CAN1实时状态 4字节 70-73
    char res70[8],res71[8],res72[8];
    hex2bin(cmd[70],res70,8);
    hex2bin(cmd[71],res71,8);
    hex2bin(cmd[72],res72,8);
    for(int i=0;i<8;i++){
        if(res70[i]=='0'){
            msg.left_status[i]=0;
        }else{
            msg.left_status[i]=1;
        }
    }
    for(int i=0;i<8;i++){
        if(res71[i]=='0'){
            msg.left_status[i+8]=0;
        }else{
            msg.left_status[i+8]=1;
        }
    }
    for(int i=0;i<8;i++){
        if(res72[i]=='0'){
            msg.left_status[i+16]=0;
        }else{
            msg.left_status[i+16]=1;
        }
    }

    // CAN2实时状态 4字节 74-77
    char res74[8],res75[8],res76[8];
    hex2bin(cmd[74],res74,8);
    hex2bin(cmd[75],res75,8);
    hex2bin(cmd[76],res76,8);
    for(int i=0;i<8;i++){
        if(res74[i]=='0'){
            msg.right_status[i]=0;
        }else{
            msg.right_status[i]=1;
        }
    }
    for(int i=0;i<8;i++){
        if(res75[i]=='0'){
            msg.right_status[i+8]=0;
        }else{
            msg.right_status[i+8]=1;
        }
    }
    for(int i=0;i<8;i++){
        if(res76[i]=='0'){
            msg.right_status[i+16]=0;
        }else{
            msg.right_status[i+16]=1;
        }
    }

    if(res70[5]=='1'||res71[1]=='1'||res74[5]=='1'||res75[1]=='1'){
        harvest_fault_state = 1;
        cout<<"驱动器警告"<<endl;
    }
    if(res70[5]=='0'&&res71[1]=='0'&&res74[5]=='0'&&res75[1]=='0'){
        harvest_fault_state = 0;
    }

    // CAN1实时故障 4字节 78-81
    char res78[8],res79[8],res80[8];
    hex2bin(cmd[78],res78,8);
    hex2bin(cmd[79],res79,8);
    hex2bin(cmd[80],res80,8);
    for(int i=0;i<8;i++){
        if(res78[i]=='0'){
            msg.left_fault[i]=0;
        }else{
            msg.left_fault[i]=1;
        }
    }
    for(int i=0;i<8;i++){
        if(res79[i]=='0'){
            msg.left_fault[i+8]=0;
        }else{
            msg.left_fault[i+8]=1;
        }
    }
    for(int i=0;i<8;i++){
        if(res80[i]=='0'){
            msg.left_fault[i+16]=0;
        }else{
            msg.left_fault[i+16]=1;
        }
    }
    // CAN2实时故障 4字节 82-85
    char res82[8],res83[8],res84[8];
    hex2bin(cmd[82],res82,8);
    hex2bin(cmd[83],res83,8);
    hex2bin(cmd[84],res84,8);
    for(int i=0;i<8;i++){
        if(res82[i]=='0'){
            msg.right_fault[i]=0;
        }else{
            msg.right_fault[i]=1;
        }
    }
    for(int i=0;i<8;i++){
        if(res83[i]=='0'){
            msg.right_fault[i+8]=0;
        }else{
            msg.right_fault[i+8]=1;
        }
    }
    for(int i=0;i<8;i++){
        if(res84[i]=='0'){
            msg.right_fault[i+16]=0;
        }else{
            msg.right_fault[i+16]=1;
        }
    }
    // 仿形 86
    
    // 备用 7字节 87-92
    // 结束符 1字节 93
    
    // while(pub.getNumSubscribers()<1);
    pub.publish(msg);
    ros::Duration(0.1).sleep();
} 