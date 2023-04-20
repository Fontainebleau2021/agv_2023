#include <ros/ros.h>
#include "agr_service/agr_service.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>

ros::ServiceClient person_client;
agr_service::agr_service srv;
ros::Publisher lio_state_pub_;
std_msgs::Int8 lio_state_;

void transshiftCallback(const std_msgs::Int8::ConstPtr& state)
{
	if (state->data == 1)
	{
		srv.request.state = "safe";
		srv.request.control  = 0;
		srv.request.planning  = 0;
    	srv.request.slam  = 0;
    	srv.request.navigation  = 0;
		ROS_INFO("Call service [state:%s, control:%d, planning:%d, slam:%d, navigation:%d]", 
			 srv.request.state.c_str(), srv.request.control, srv.request.planning, srv.request.slam, srv.request.navigation);
		person_client.call(srv);
		sleep(3);
		person_client.call(srv);
		sleep(3);
		srv.request.slam  = 1;
    	srv.request.navigation  = 5;
		person_client.call(srv);
		ROS_INFO("Call service [state:%s, control:%d, planning:%d, slam:%d, navigation:%d]", 
			 srv.request.state.c_str(), srv.request.control, srv.request.planning, srv.request.slam, srv.request.navigation);
		sleep(5);
		lio_state_.data = 1;
		lio_state_pub_.publish(lio_state_);
	}
	else if (state->data == 2)
	{
		srv.request.state = "safe";
		srv.request.control  = 0;
		srv.request.planning  = 0;
    	srv.request.slam  = 0;
    	srv.request.navigation  = 4;
		ROS_INFO("Call service [state:%s, control:%d, planning:%d, slam:%d, navigation:%d]", 
			 srv.request.state.c_str(), srv.request.control, srv.request.planning, srv.request.slam, srv.request.navigation);
		person_client.call(srv);
		sleep(5);
		person_client.call(srv);
		sleep(5);
    	srv.request.navigation  = 9;
		person_client.call(srv);
		ROS_INFO("Call service [state:%s, control:%d, planning:%d, slam:%d, navigation:%d]", 
			 srv.request.state.c_str(), srv.request.control, srv.request.planning, srv.request.slam, srv.request.navigation);
		sleep(5);
		lio_state_.data = 2;
		lio_state_pub_.publish(lio_state_);
	}
	else if (state->data == 3)
	{
		srv.request.state = "safe";
		srv.request.control  = 0;
		srv.request.planning  = 0;
    	srv.request.slam  = 0;
    	srv.request.navigation  = 8;
		ROS_INFO("Call service [state:%s, control:%d, planning:%d, slam:%d, navigation:%d]", 
			 srv.request.state.c_str(), srv.request.control, srv.request.planning, srv.request.slam, srv.request.navigation);
		person_client.call(srv);
		sleep(3);
		person_client.call(srv);
		sleep(3);
    	srv.request.slam  = 1;
    	srv.request.navigation  = 13;
		person_client.call(srv);
		ROS_INFO("Call service [state:%s, control:%d, planning:%d, slam:%d, navigation:%d]", 
			 srv.request.state.c_str(), srv.request.control, srv.request.planning, srv.request.slam, srv.request.navigation);
		sleep(5);
		lio_state_.data = 3;
		lio_state_pub_.publish(lio_state_);
	}
	else if (state->data == 4)
	{
		srv.request.state = "safe";
		srv.request.control  = 0;
		srv.request.planning  = 0;
    	srv.request.slam  = 0;
    	srv.request.navigation  = 12;
		ROS_INFO("Call service [state:%s, control:%d, planning:%d, slam:%d, navigation:%d]", 
			 srv.request.state.c_str(), srv.request.control, srv.request.planning, srv.request.slam, srv.request.navigation);
		person_client.call(srv);
		sleep(5);
		person_client.call(srv);
		sleep(5);
    	srv.request.navigation  = 1;
		person_client.call(srv);
		ROS_INFO("Call service [state:%s, control:%d, planning:%d, slam:%d, navigation:%d]", 
			 srv.request.state.c_str(), srv.request.control, srv.request.planning, srv.request.slam, srv.request.navigation);
		sleep(5);
		lio_state_.data = 0;
		lio_state_pub_.publish(lio_state_);
	}
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
	ros::init(argc, argv, "agr_service_client");

    // 创建节点句柄
	ros::NodeHandle node;

    // 发现/spawn服务后，创建一个服务客户端，连接名为/command的service
	ros::service::waitForService("/command");
	person_client = node.serviceClient<agr_service::agr_service>("/command");

    // 初始化learning_service::Person的请求数据
	srv.request.state = "safe";
	srv.request.control  = 0;
	srv.request.planning  = 0;
    srv.request.slam  = 0;
    srv.request.navigation  = 1;

    // 请求服务调用
	ROS_INFO("Call service [state:%s, control:%d, planning:%d, slam:%d, navigation:%d]", 
			 srv.request.state.c_str(), srv.request.control, srv.request.planning, srv.request.slam, srv.request.navigation);
	person_client.call(srv);

	// 显示服务调用结果
	ROS_INFO("Show  result : %s", srv.response.result.c_str());

	ros::Subscriber trans_sub_ = node.subscribe<std_msgs::Int8>("/trans_shift", 10, &transshiftCallback);
	lio_state_pub_ = node.advertise<std_msgs::Int8>("/lio_state", 10);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

	return 0;
};
