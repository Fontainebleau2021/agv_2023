#ifndef HAVESTINGROBOTCLIENT_H
#define HAVESTINGROBOTCLIENT_H
#pragma once
#include <iostream>
#include "robotsocket/robotsocket.h"
using namespace std;

class HarvestingRobotClient:public RobotSocket{
    public:

        unsigned char harvestcmd[13]={0x3c, 0x30, 0x30, 0x7c, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3e};
        unsigned char harvestrecv[94];
        // 功能00：心跳指令
        unsigned char harvestcmd0[13]={0x3c, 0x30, 0x30, 0x7c, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3e};
        
        //  功能01：推杆伸出<01|10000000>；推杆缩回<01|20000000>；停止<01|00000000>
        unsigned char harvestcmd1_long[13] = {0x3c, 0x30, 0x31, 0x7c, 0x31, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3e};
        unsigned char harvestcmd1_short[13] = {0x3c, 0x30, 0x31, 0x7c, 0x32, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3e};
        unsigned char harvestcmd1_stop[13] = {0x3c, 0x30, 0x31, 0x7c, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3e};
        
        // 功能02：电源开<02|10000000>；电源关<02|00000000>
        unsigned char harvestcmd2_on[13] = {0x3c, 0x30, 0x32, 0x7c, 0x31, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3e};
        unsigned char harvestcmd2_off[13] = {0x3c, 0x30, 0x32, 0x7c, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3e};
        
        // 功能03：整体上升<03|10000000>；上升停止<03|00000000>
        unsigned char harvestcmd3_up[13 ] = {0x3c, 0x30, 0x33, 0x7c, 0x31, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3e};
        unsigned char harvestcmd3_up_stop[13] = {0x3c, 0x30, 0x33, 0x7c, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3e};
       
        // 功能04：整体下降<04|10000000>；下降停止<04|00000000>
        unsigned char harvestcmd4_down[13] = {0x3c, 0x30, 0x34, 0x7c, 0x31, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3e};
        unsigned char harvestcmd4_down_stop[13] = {0x3c, 0x30, 0x34, 0x7c, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3e};

        // 功能05：设置两个履带电机<05|左电机转速右电机转速>
        unsigned char harvestcmd5[13] = {0x3c, 0x30, 0x35, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x3e};
        
        // 功能06：设置割刀和输送电机电压<06|割刀电机电压输送电机电压> 1v 1v测试
        unsigned char harvestcmd6[13] = {0X3c, 0x30, 0x36, 0x7c, 0x8c, 0xcc, 0x8c, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x3e};
        
        // 功能07：设置输送带电机<07|输送带电机电压备用> 1v测试
        unsigned char harvestcmd7[13] = {0X3c, 0x30, 0x36, 0x7c, 0x8c, 0xcc, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e};
        
        // 指令8：延长电机DO8使能（弃用，因为机械部分拆除）
        unsigned char harvestcmd8[13];
        
        // 指令9：驱动器复位<09|00000000>
        unsigned char harvestcmd9[13] = {0x3c, 0x30, 0x39, 0x7c, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3e};
        
        // 指令10：驱动器初始化<10|00000000>
        unsigned char harvestcmd10[13] = {0x3c, 0x31, 0x30, 0x7c, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3e};
        
        // 指令11：ARM软件复位<11|00000000>
        unsigned char harvestcmd11[13] = {0x3c, 0x31, 0x31, 0x7c,  0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3e};
        
        // 指令13 ：使用仿形功能<13|10000000>  <13|00000000>
        unsigned char harvestcmd13_on[13] = {0x3c, 0x31, 0x33, 0x7c,  0x31, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3e};
        unsigned char harvestcmd13_off[13] = {0x3c, 0x31, 0x33, 0x7c,  0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3e};

        // 心跳state=0; action state=1; vel state=2;
        int harvest_state = 0;
        // 推杆 0停止 1伸出 2缩回
        int harvest_push_rod = 0;
        // 平台 1上升 2上升停止 3下降 4下降停止
        int harvest_platform = 0;
        // 左右电机转速r/min
        int harvest_left_vel = 0;
        int harvest_right_vel = 0;
        // 割刀 电压0-10V
        double harvest_knife = 0;
        // 前端输送电机 电压0-10V
        double harvest_shake = 0;
        // 传送带 电压0-10V
        double harvest_conveyor = 0;
        // 仿形 0禁用，1使用
        int harvest_imitate = 0;
        // 过热状态值，0正常，1正处于散热状态
        int harvest_temp_state=0;
        // 错误状态值，0正常，1正处于错误清除过程
        int harvest_fault_state=0;
        // 传送带状态值，1触碰下限位需要上升，2触碰上限位需要下降，0保持不变。
        int harvest_platform_state=0;
        
        // int转16进制char[]
        void int2hex(int i, int length, char * hex);
        // 16进制转2进制
        void hex2bin(unsigned char in, char *bin_str, int str_size);
        // 16进制char转10进制int
        int hexChar2decInt(unsigned char* hex_char, int length);
        
        // 左右轮电压解析函数，输入左右轮转速，输出发送的命令
        void harvest_vel_cmd(double left_vel, double right_vel, unsigned  char* cmd);
        // 收割装置电压解析函数，输入割刀电压、前端输送电机电压，输出发送的命令
        void harvest_cut_device_cmd(double knife_voltage, double shake_voltage, unsigned  char* cmd);
        // 传送带电压解析函数，输入传送带电压，输出发送的命令
        void harvest_conveyor_cmd(double voltage, unsigned char* cmd);
        
        // 接受命令，解析命令，发布ROS话题/harvest_state
        void harvest_pub(unsigned char * cmd, int cmd_length);
};

#endif