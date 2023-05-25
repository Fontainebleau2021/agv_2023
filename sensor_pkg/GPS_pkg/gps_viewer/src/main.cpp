#include <ros/ros.h>;
#include "turtlesim/Pose.h"
#include <sensor_msgs/NavSatFix.h>;
#include <geometry_msgs/PoseStamped.h>;
#include <nav_msgs/Path.h>;
#include <math.h>;
#include <iostream>
struct my_pose
{    
	double latitude;    
	double longitude;    
	double altitude;
};

//角度制转弧度制
double rad(double d) 
{	
	return d * 3.1415926 / 180.0;
}
//全局变量
static double EARTH_RADIUS = 6378.137;//地球半径
ros::Publisher state_pub_,map_pub_;
ros::Publisher pose_pub_,mappose_pub_;
nav_msgs::Path ros_path_,map_path_;
bool init;
my_pose init_pose;

void Rot(double x1, double y1, double alpha, double& x2, double& y2)
{
	x2 = x1 * cos(alpha) - y1 * sin(alpha);
	y2 = x1 * sin(alpha) + y1 * cos(alpha);
}


void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr)
{    
	//初始化    
	if(!init)    
	{        
		//init_pose.latitude = gps_msg_ptr->latitude;
        //init_pose.longitude = gps_msg_ptr->longitude;
		init_pose.latitude = 31.64957787883;
		init_pose.longitude = 121.5906370503;
	    init_pose.altitude = gps_msg_ptr->altitude;
		std::cout<<"gps_msg_ptr->latitude:"<<std::setprecision(15)<<gps_msg_ptr->latitude<<std::endl;
		std::cout<<"gps_msg_ptr->longitude:"<<std::setprecision(16)<<gps_msg_ptr->longitude<<std::endl;
	    init = true;    
	}
	else
	{
		//计算相对位置
	        double radLat1 ,radLat2, radLong1,radLong2,delta_lat,delta_long,delta_x,delta_y;		
			radLat1 = rad(init_pose.latitude);
	        radLong1 = rad(init_pose.longitude);
			radLat2 = rad(gps_msg_ptr->latitude);
			radLong2 = rad(gps_msg_ptr->longitude);

	        //计算x
			delta_lat = radLat2 - radLat1;
	        delta_long = 0;
			delta_x = delta_lat;
	        double x = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat1 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ));
	        x = x*EARTH_RADIUS*1000;

	        //计算y
			delta_lat = 0;
	        delta_long = radLong1  - radLong2;
			delta_y = delta_long;
	        double y = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat2 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ) );
	        y = y*EARTH_RADIUS*1000;

	        //计算z
	        double z = gps_msg_ptr->altitude - init_pose.altitude;

	        //发布轨迹
	        ros_path_.header.frame_id = "world";
	        ros_path_.header.stamp = ros::Time::now();
			map_path_.header.frame_id = "map";
	        map_path_.header.stamp = ros::Time::now();

	        geometry_msgs::PoseStamped pose,map_pose;
	        pose.header = ros_path_.header;
			map_pose.header = map_path_.header;
			if((delta_x<0)and(delta_y<0))
			{
				pose.pose.position.x = y;
				pose.pose.position.y = -x;
			}
			else if((delta_x>0)and(delta_y<0))
			{
				pose.pose.position.x = y;
				pose.pose.position.y = x;
			}
			else if((delta_x>0)and(delta_y>0))
			{
				pose.pose.position.x = -y;
				pose.pose.position.y = x;
			}
			else if((delta_x<0)and(delta_y>0))
			{
				pose.pose.position.x = -y;
				pose.pose.position.y = -x;
			}
			else
			{
				pose.pose.position.x = x;
				pose.pose.position.y = y;
			}
		//pose.pose.position.z = z;
		pose.pose.position.z = 0;
		map_pose.pose.position.z = 0;
	    ros_path_.poses.push_back(pose);
	    //ROS_INFO("( x:%0.6f ,y:%0.6f ,z:%0.6f)",x ,y ,z );
		//std::cout<<"( x:"<<x<<"),(y:"<<y<<"),(z:"<<z<<")"<<std::endl;
		double gps_x,gps_y,map_x,map_y;
		gps_x = pose.pose.position.x;
		gps_y = pose.pose.position.y;
		Rot(gps_x,gps_y,-1.470795,map_x,map_y);
		map_pose.pose.position.x = map_x;
		map_pose.pose.position.y = map_y;
		map_path_.poses.push_back(map_pose);
	    state_pub_.publish(ros_path_);
		pose_pub_.publish(pose);
		map_pub_.publish(map_path_);
		mappose_pub_.publish(map_pose);
	    }
	}

int main(int argc,char **argv)
{
    init = false;
    ros::init(argc,argv,"gps_subscriber");
    ros::NodeHandle n;
    ros::Subscriber pose_sub=n.subscribe("/fix",10,gpsCallback);

    state_pub_ = n.advertise<nav_msgs::Path>("gps_path", 10);
	pose_pub_ = n.advertise<geometry_msgs::PoseStamped>("gps_pose", 10);
	map_pub_ = n.advertise<nav_msgs::Path>("map_path", 10);
	mappose_pub_ = n.advertise<geometry_msgs::PoseStamped>("map_pose", 10);

    ros::spin();
    return 0;
}
