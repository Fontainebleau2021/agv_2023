#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <string.h>
#include <iostream>

#include "robotsocket/robotsocket.h"
#include <thread>
#include <atomic>

#define MYPORT 3014	  // 正式端口号
const char *SERVER_IP = "101.231.56.118"; // 正式ip

// #define MYPORT 3311	  // 测试端口号
// const char *SERVER_IP = "117.144.159.178"; // 测试ip

// #define MYPORT 3311	  // 本地端口号
// const char *SERVER_IP = "127.0.0.1"; //本地ip

#define BUF_SIZE 1024 // 数据缓冲区最大长度

int recvbuf_length; // 接收数据的长度
int send_length;	// 发送数据长度

using namespace std;

int main(int argc, char **argv)
{
	// 初始化ros节点
	ros::init(argc, argv, "robotsocket");
	ros::NodeHandle n;

	char recvbuf[BUF_SIZE];
	char recvbuf2[BUF_SIZE];

	/*
	 *@fuc: socket()创建套节字
	 *
	 */
	int socket_cli = socket(AF_INET, SOCK_STREAM, 0);
	if (socket_cli < 0)
	{
		std::cout << "socket() error\n";
		return -1;
	}

	/*
	 *@fuc: 服务器端IP4地址信息,struct关键字可不写
	 *@fuc: 初始化sever地址信息
	 */
	struct sockaddr_in sev_addr;
	memset(&sev_addr, 0, sizeof(sev_addr));
	sev_addr.sin_family = AF_INET;
	sev_addr.sin_port = htons(MYPORT);
	sev_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
	std::cout << "connecting..." << std::endl;
	/*
	 *@fuc: 使用connect()函数来配置套节字,建立一个与TCP服务器的连接
	 */
	if (connect(socket_cli, (struct sockaddr *)&sev_addr, sizeof(sev_addr)) < 0)
	{
		std::cout << "connect error" << std::endl;
		return -1;
	}
	else
		std::cout << "connected successfullly!" << std::endl;

	RobotSocket robotsocket;

	// 线程1：监听机器人ros消息
	thread sub([&]
			   { robotsocket.robot_sub(); });

	sleep(1);

	while (ros::ok())
	{
		// 使用recv()函数来接收服务器发送的消息
		recvbuf_length = recv(socket_cli, recvbuf, sizeof(recvbuf), 0);
		
		if (recvbuf_length >= 5)
		{
			// 显示接受到的消息recvbuf
			cout << "recvbuf:";
			robotsocket.display(recvbuf, recvbuf_length);

			// 测试指令
			char recv1[] = {0x00, 0x00, 0x00, 0x00, 0x01};
			// recv2: 00 00 00 00 02 + {"x":"7","y":"7","id":"1"}
			char recv2[] = {0x00, 0x00, 0x00, 0x00, 0x02, 0x7B, 0x22, 0x78, 0x22, 0x3A, 0x22, 0x37, 0x22, 0x2C, 0x22, 0x79, 0x22, 0x3A, 0x22, 0x37, 0x22, 0x2C, 0x22, 0x69, 0x64, 0x22, 0x3A, 0x22, 0x31, 0x22, 0x7D};
			// recv2: 00 00 00 00 02 + {"x":"7.0123456","y":"7","id":"1"}
			// char recv2[] = {0x00, 0x00, 0x00, 0x00, 0x02, 0x7b, 0x22,0x78, 0x22, 0x3a, 0x22, 0x37, 0x2e, 0x30,0x31, 0x32, 0x33, 0x34, 0x35, 0x36,0x22, 0x2c, 0x22, 0x79, 0x22, 0x3a, 0x22, 0x37, 0x22, 0x2c, 0x22, 0x69, 0x64, 0x22, 0x3a, 0x22, 0x31, 0x22, 0x7d};
			char recv3[] = {0x00, 0x00, 0x00, 0x00, 0x03};
			char recv4[] = {0x00, 0x00, 0x00, 0x00, 0x04};
			char recv5[] = {0x00, 0x00, 0x00, 0x00, 0x05};
			char recv6[] = {0x00, 0x00, 0x00, 0x00, 0x06};
			char recv7[] = {0x00, 0x00, 0x00, 0x00, 0x07};
			char recv8[] = {0x00, 0x00, 0x00, 0x00, 0x08};
			char recv9[] = {0x00, 0x00, 0x00, 0x00, 0x09};

			// for(int i =0;i<sizeof(recv2);i++){
			// 	recvbuf[i]=recv2[i];
			// };
			// recvbuf_length=sizeof(recv2);
			// recvbuf[4] = recv6[4];

			// 匹配指令内容
			switch (recvbuf[4])
			{

			case 0x01: // 连接
			{
				cout << "received recv1" << endl;
				send(socket_cli, robotsocket.cmd1, sizeof(robotsocket.cmd1), 0);
				cout << "send cmd1 successfully" << endl;
				break;
			}

			case 0x02: // 运动指令
			{
				// 先把xy坐标发布给机器人
				// 机器人收到坐标，改变状态为0，开始执行作业指令，while循环开始
				// 机器人作业结束，发布状态为1，while循环结束，发布结束指令。
				// 停止的指令优先级最高，点击停止的时候，状态改变，所以还是要多线程，同时监听状态改变
				robotsocket.send_signal = true;
				robotsocket.listen_signal = true;
				robotsocket.working_signal = true;

				// 第一次发送
				cout << "received recv2" << endl;
				memset(robotsocket.cmd2, 0, sizeof(robotsocket.cmd2));								// 清除内存
				send_length = robotsocket.action2_start(recvbuf, robotsocket.cmd2, recvbuf_length); // 操作2
				send(socket_cli, robotsocket.cmd2, send_length, 0);									// 发送
				cout << "send cmd2 successfully" << endl;
				robotsocket.robot_pub(); // 发布目的地给机器人

				// 线程1:发送
				thread send_cmd([&]
								{
				while(robotsocket.send_signal.load()){
					//发送
					memset(robotsocket.cmd2, 0, sizeof(robotsocket.cmd2));//清除内存
					send_length = robotsocket.action2(recvbuf,robotsocket.cmd2,recvbuf_length);//操作2
					send(socket_cli, robotsocket.cmd2, send_length, 0);//发送
					cout << "send cmd2 successfully" << endl;
					sleep(1);
					//如果有工作完成信号
					if(robotsocket.working_signal == false){
						robotsocket.listen_signal = false; // 监听停止
						robotsocket.send_signal = false; // 发送停止
					}
				}
				cout<<"thread1 killed"<<endl; });

				// 线程2:监听停止信号
				thread listen_cmd([&]
								  {
				while(robotsocket.listen_signal.load()){
					recv(socket_cli, recvbuf2, sizeof(recvbuf2), MSG_DONTWAIT); // recv()非阻塞
					//如果有停止信号
					if(recvbuf2[4]==0x04){
						robotsocket.listen_signal = false; // 监听停止
						robotsocket.send_signal = false; // 发送停止
						// robotsocket.working_signal == false;
						robotsocket.stop(); //ros发布停止话题
					}
				}
				cout<<"thread2 killed"<<endl; });

				// 线程join
				send_cmd.join();
				listen_cmd.join();

				// 结束发送
				memset(robotsocket.cmd2, 0, sizeof(robotsocket.cmd2));							   // 清除内存
				send_length = robotsocket.action2_over(recvbuf, robotsocket.cmd2, recvbuf_length); // 操作2_over
				send(socket_cli, robotsocket.cmd2, send_length, 0);								   // 发送
				cout << "send cmd2_over successfully" << endl;
				break;
			}

			case 0x03: // 获取机器人位置
			{
				cout << "received recv3" << endl;
				memset(robotsocket.cmd3, 0, sizeof(robotsocket.cmd3));						  // 清除内存
				send_length = robotsocket.action3(recvbuf, robotsocket.cmd3, recvbuf_length); // 操作3
				send(socket_cli, robotsocket.cmd3, send_length, 0);							  // 发送
				cout << "send cmd3 successfully" << endl;
				break;
			}

			case 0x04: // 停止
			{
				cout << "received recv4" << endl;
				robotsocket.stop();															  // ros发布停止话题
				memset(robotsocket.cmd4, 0, sizeof(robotsocket.cmd4));						  // 清除内存
				send_length = robotsocket.action4(recvbuf, robotsocket.cmd4, recvbuf_length); // 操作4
				send(socket_cli, robotsocket.cmd4, send_length, 0);							  // 发送
				cout << "send cmd4 successfully" << endl;
				break;
			}

			case 0x05: // 入库
			{
				cout << "received recv5" << endl;
				robotsocket.warehouse();													  // ros发布入库话题
				memset(robotsocket.cmd4, 0, sizeof(robotsocket.cmd4));						  // 清除内存
				send_length = robotsocket.action4(recvbuf, robotsocket.cmd4, recvbuf_length); // 操作4
				send(socket_cli, robotsocket.cmd4, send_length, 0);							  // 发送
				cout << "send cmd5 successfully" << endl;
				break;
			}

			case 0x06: // 前进
			{
				cout << "received recv6" << endl;
				robotsocket.forward();														  // ros前进速度发布函数，延时1s停止。
				memset(robotsocket.cmd4, 0, sizeof(robotsocket.cmd4));						  // 清除内存
				send_length = robotsocket.action4(recvbuf, robotsocket.cmd4, recvbuf_length); // 操作4
				send(socket_cli, robotsocket.cmd4, send_length, 0);							  // 发送
				cout << "send cmd6 successfully" << endl;
				break;
			}

			case 0x07: // 后退
			{
				cout << "received recv7" << endl;
				robotsocket.back();															  // ros后退速度发布函数，延时1s停止。
				memset(robotsocket.cmd4, 0, sizeof(robotsocket.cmd4));						  // 清除内存
				send_length = robotsocket.action4(recvbuf, robotsocket.cmd4, recvbuf_length); // 操作4
				send(socket_cli, robotsocket.cmd4, send_length, 0);							  // 发送
				cout << "send cmd7 successfully" << endl;
				break;
			}

			case 0x08: // 左转
			{
				cout << "received recv8" << endl;
				robotsocket.left();															  // ros左转速度发布函数，延时1s停止。
				memset(robotsocket.cmd4, 0, sizeof(robotsocket.cmd4));						  // 清除内存
				send_length = robotsocket.action4(recvbuf, robotsocket.cmd4, recvbuf_length); // 操作4
				send(socket_cli, robotsocket.cmd4, send_length, 0);							  // 发送
				cout << "send cmd8 successfully" << endl;
				break;
			}

			case 0x09: // 右转
			{
				cout << "received recv9" << endl;
				robotsocket.right();														  // ros右转速度发布函数，延时1s停止。
				memset(robotsocket.cmd4, 0, sizeof(robotsocket.cmd4));						  // 清除内存
				send_length = robotsocket.action4(recvbuf, robotsocket.cmd4, recvbuf_length); // 操作4
				send(socket_cli, robotsocket.cmd4, send_length, 0);							  // 发送
				cout << "send cmd9 successfully" << endl;
				break;
			}

			default:
				cout << "no this cmd" << endl;
				break;
			}
		}
	}

	// 加入线程
	sub.join();

	/*
	 *@fuc: 关闭连接
	 */
	close(socket_cli);
	return 0;
}
