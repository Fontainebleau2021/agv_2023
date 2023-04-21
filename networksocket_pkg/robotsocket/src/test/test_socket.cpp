#include <iostream>
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

#define MYPORT 8081	  // 端口号
#define BUF_SIZE 1024 // 数据缓冲区最大长度

char *SERVER_IP = "106.15.233.31";

using namespace std;

int main(int argc, char **argv)
{

	char recvbuf[BUF_SIZE];

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

	cout << "连接成功" << endl;
	char cmd[] = "{\"lng\":121,\"lat\":36}";
	send(socket_cli, cmd, sizeof(cmd), 0);//发送
	// recv(socket_cli, recvbuf, sizeof(recvbuf), 0);
	cout << cmd << endl;
	// cout << strlen(cmd) << endl;
	cout << "发送成功" << endl;
	/*
	 *@fuc: 关闭连接
	 */
	close(socket_cli);
	return 0;
}