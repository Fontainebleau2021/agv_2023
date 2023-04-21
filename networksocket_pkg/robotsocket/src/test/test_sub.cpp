#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include"nav_msgs/Path.h"
#include"robotsocket/state.h"
#include <iostream>
using namespace std;

double x,y,lon,lat;
int state;

void subscriberCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
        // printf("latitude=%.10f, longitude=%.10f\n",msg->latitude,msg->longitude);
        lat = msg->latitude;
        lon = msg->longitude;
}

void subscriberCallback2(const nav_msgs::Path::ConstPtr& path){
        // printf("x=%f, y=%f\n",path->poses.back().pose.position.x,path->poses.back().pose.position.y);
        x = path->poses.back().pose.position.x;
        y = path->poses.back().pose.position.y;
}

void subscriberCallback3(const robotsocket::state::ConstPtr& msg){
        // printf("state=%d\n", msg->state);
        state = msg->state;
}


int main(int argc, char **argv)
{
        ros::Subscriber subscriber;
        ros::Subscriber subscriber2;
        ros::Subscriber subscriber3;
        ros::init(argc, argv,"sub");
        ros::NodeHandle nd;
        subscriber = nd.subscribe("/fix", 10, &subscriberCallback);
        subscriber2 = nd.subscribe("/gps_path", 10, &subscriberCallback2);
        subscriber3 = nd.subscribe("/state", 10, &subscriberCallback3);
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
               loop_rate.sleep();
                ros::spinOnce();
                printf("latitude=%.10f, longitude=%.10f\n",lat,lon);
                printf("x=%f, y=%f\n",x,y);
                printf("state=%d\n", state);
        }

        return 0;
}
