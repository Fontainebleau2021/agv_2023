#include <ros/ros.h>
#include "agr_service/agr_service.h"

int main(int argc, char** argv)
{
    // 初始化ROS节点
	ros::init(argc, argv, "agr_service_client");

    // 创建节点句柄
	ros::NodeHandle node;

    // 发现/spawn服务后，创建一个服务客户端，连接名为/command的service
	ros::service::waitForService("/command");
	ros::ServiceClient person_client = node.serviceClient<agr_service::agr_service>("/command");

    // 初始化learning_service::Person的请求数据
	agr_service::agr_service srv;
	srv.request.state = "safe";
	srv.request.control  = 0;
	srv.request.planning  = 0;
    srv.request.slam  = 0;
    srv.request.gps  = 0;

    // 请求服务调用
	ROS_INFO("Call service [state:%s, control:%d, planning:%d, slam:%d, gps:%d]", 
			 srv.request.state.c_str(), srv.request.control, srv.request.planning, srv.request.slam, srv.request.gps);

	person_client.call(srv);

	// 显示服务调用结果
	ROS_INFO("Show  result : %s", srv.response.result.c_str());

	return 0;
};
