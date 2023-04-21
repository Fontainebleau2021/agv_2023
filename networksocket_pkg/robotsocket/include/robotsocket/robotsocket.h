#ifndef ROBOTSOCKET_H
#define ROBOTSOCKET_H
#pragma once
#include <iostream>
#include <string.h>
#include <atomic>
#include "robotsocket/state.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Int8.h"
using namespace std;
 
 class RobotSocket{
    public:

        stringstream position_x, position_y, position_lon, position_lat, ss_state;
        string str_x, str_y, str_lon, str_lat, str_state;
        
        // 展示接受和发送数据内容
        void display(char* recv, int length){
            for (int i = 0; i < length; i++)
		    {
			    printf("%02x ", recv[i]&0xFF);
		    };
            cout << endl;
        };

        void display(unsigned char* recv, int length){
            for (int i = 0; i < length; i++)
		    {
			    printf("%02x ", recv[i]&0xFF);
		    };
            cout << endl;
        };
        
        //转换函数
        void array2char(char* dbuff, char* sbuff); // 把char[]拼成char
        int char2array(char *hex, char *bytes);   // 把char拆开成char[]
        int hex2ascii(char* hex, char* ascii); // 16进制转ascii
        void ascii2hex(char* ascii, char* hex); // ascii转16进制

        // 接受到recv1:00 00 00 00 01 (心跳指令)
		// 回复cmd1(机器人编号):50 00 00 01
        // 0x00, 0x32, 0x00, 0x00, 0x03
		const char cmd1[5] = {0x00, 0x32, 0x00, 0x00, 0x03};

        // 接受到recv2:00 00 00 00 02 + {"x":"7","y":"7","id":"1"} (机器人下发任务指令)
		// 回复cmd2: 00 10 06 07 30 01 00  + {"runningSpeed":"1","recoveryRate":"60","x":"1","y":"1","lon":"1","id":"1","lat":"1","status":"01"}
		const char cmd2_head[7] = {0x00, 0x32, 0x00, 0x00, 0x03, 0x01, 0x00};
        const char cmd2_over[7] = {0x00, 0x32, 0x00, 0x00, 0x03, 0x01, 0x01};
        char cmd2[1024];// 发送给服务器的指令

        void recv2_json(char * str,char * myJsonChar);// 02指令的json处理函数
        void recv2_json_start(char * str,char * myJsonChar);// 02指令的json处理函数
        void recv2_json_over(char * str,char * myJsonChar);// 02指令的json处理函数

        int action2_start(char * recv2, char * recv2_send,int recv2_length);   //02指令操作函数
        int action2(char * recv2, char * recv2_send,int recv2_length);   //02指令操作函数
         int action2_over(char * recv2, char * recv2_send,int recv2_length);   //02指令操作函数          

        //接受到recv3:00 00 00 00 03 (实时获取机器人的位置指令)
        //回复cmd3: 00 10 06 07 30 00 + {"runningSpeed":"1","recoveryRate":"60","x":"0","y":"0","lon":"0","lat":"0","status":"02"} 
        const char cmd3_head[6] = {0x00, 0x32, 0x00, 0x00, 0x03, 0x00};
        char cmd3[1024]; // 发送给服务器的指令
        void recv3_json(char * myJsonChar); // 03指令的json处理函数    
        int action3(char * recv3, char * recv3_send,int recv3_length); //03指令操作函数       

        //接收到recv4:00 00 00 00 04(停止)
        //接收到recv5:00 00 00 00 05(入库)
        //接收到recv6:00 00 00 00 06(前进)
        //接收到recv7:00 00 00 00 07(后退)
        //接收到recv7:00 00 00 00 08(左转)
        //接收到recv9:00 00 00 00 09(右转)
        //都回复cmd4:00 10 06 07 30 02 +{"runningSpeed":"8.6","recoveryRate":"0","x":"5","y":"5","lon":"5","lat":"5","status":"01"}
        const char cmd4_head[6] = {0x00, 0x32, 0x00, 0x00, 0x03, 0x02};
        char cmd4[1024];//发送给服务器的指令
        void recv4_json(char * myJsonChar); // 06指令的json处理函数
        int action4(char * recv4, char * recv4_send,int recv4_length);//06指令操作函数
       
       //机器人控制
        void forward(); //ros前进指令
        void back(); //ros后退指令
        void left(); //ros左转指令
        void right(); //ros右转指令
        void stop();// 停止
        void warehouse();// 入库

        atomic_bool working_signal;
        atomic_bool send_signal;
        atomic_bool listen_signal;
        
        double target[2]; // target_x, target_y
        double position[4] = {}; // x, y, lon, lat

        //发布目的地给机器人target_x, target_y
        void robot_pub();
        
        //订阅机器人参数x y lon lat state
        void robot_sub();
        int cmd_state=1;
        atomic_int state;
        //订阅机器人状态
        void state_sub(atomic_bool * working_signal);
        // 回调函数
        void subscriberCallback_state(const robotsocket::state::ConstPtr& msg);
        void subscriberCallback_gps(const sensor_msgs::NavSatFix::ConstPtr& gps);
        void subscriberCallback_path(const nav_msgs::Path::ConstPtr& path);
        void subscriberCallback_cmd_state(const std_msgs::Int8::ConstPtr& state);


 };

#endif