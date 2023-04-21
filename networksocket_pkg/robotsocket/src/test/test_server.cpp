#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#define PORT 3311    //端口号
#define LOG  1       //请求队列中最大连接数量

using namespace std;
char recvbuf[1024]; // 接收数据的长度
int recv_length;

unsigned char send_cmd[94] = {0x3C, 0xAF, 0xCF, 0x1F, 0xE9, 0x1F, 0xC0, 0x10, 0xE2, 0x00, 0x18, 0xFF, 0xF2, 0xFF, 0xF6, 0x1A, 0x6E, 0x1A, 0x6D, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF9, 0xFD, 0x56, 0x00, 0xFF, 0xFF, 0x02, 0x03, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x00, 0x17, 0x00, 0x18, 0x00, 0x16, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E};
char recv1[] = {0x00, 0x00, 0x00, 0x00, 0x01};
int main (int argc, char** argv)
{
	ros::init(argc, argv, "server_node");
	ros::NodeHandle nh;
	
	/*
	 *@fuc: 监听套节字描述符和连接套节字描述符
	 *@fuc; 服务器端和客户端IP4地址信息,struct关键字可不要
	 */
	int listenfd, connectfd;
	struct sockaddr_in sever;
	struct sockaddr_in client;
	socklen_t addrlen;
	
	/*
	 *@fuc: 使用socket()函数产生套节字描述符
	 */
	listenfd = socket(AF_INET, SOCK_STREAM, 0);
	if(listenfd == -1)
	{
		printf("socket() error\n");
		return -1;
	}
	
	/*
	 *@fuc: 初始化server套节字地址信息 
	 */
	memset((void *)&sever,0,sizeof(sever));
	sever.sin_family = AF_INET;
	sever.sin_addr.s_addr = htonl(INADDR_ANY);
	sever.sin_port = htons(PORT);
 
 	/*
	 *@fuc: 用bind()函数，将套接字与指定的协议地址绑定 
	 */
	if(bind(listenfd,(struct sockaddr *)&sever,sizeof(sever)) < 0)
	{
		printf("bind() error\n");
		return -1;
	}
	
 	/*
	 *@fuc: 使用listen()函数，等待客户端的连接 
	 */
    if(listen(listenfd, LOG) < 0)
    {
        printf("listen() error.\n");
        return -1;
    }
    
    addrlen = sizeof(client);
    
    //不断监听客户端请求
	while(ros::ok)
	{
		connectfd = accept(listenfd,(struct sockaddr *)&client,&addrlen);
		if(connectfd < 0)
		{
			printf("connect() error \n");
			return -1;
		}
		printf("You got a connection from client's IP is %s, port is %d\n",
				inet_ntoa(client.sin_addr), ntohs(client.sin_port));
		while(1){
			send(connectfd,recv1,sizeof(recv1),0);
			recv_length = recv(connectfd, recvbuf, sizeof(recvbuf), 0);
			cout<<"recv: ";
			for (int i = 0; i < recv_length; i++)
			{
				printf("%02x ", recvbuf[i]&0xFF);
			};
			cout << endl;
		}
		
		close(connectfd);
	}
	close(listenfd);
	return 0;
}
