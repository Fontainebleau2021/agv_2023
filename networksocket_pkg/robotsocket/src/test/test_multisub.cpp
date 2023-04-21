#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/thread/thread.hpp>
#include <iostream>

#include "sensor_msgs/NavSatFix.h"
#include"nav_msgs/Path.h"

void multi_callback(const sensor_msgs::NavSatFix::ConstPtr& gps, const nav_msgs::Path::ConstPtr& path)
{
    printf("latitude=%.10f, longitude=%.10f\n",gps->latitude,gps->longitude);
    printf("x=%f, y=%f\n",path->poses.back().pose.position.x,path->poses.back().pose.position.y);
    std::cout << "同步完成！" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_sub");
    ros::NodeHandle n;

    message_filters::Subscriber<sensor_msgs::NavSatFix> subscriber_gps(n,"/fix",1000,ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<nav_msgs::Path> subscriber_path(n,"/gps_path",1000,ros::TransportHints().tcpNoDelay());
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, nav_msgs::Path> syncPolicy;
    //message_filters::TimeSynchronizer<sensor_msgs::LaserScan,geometry_msgs::PoseWithCovarianceStamped> sync(subscriber_laser, subscriber_pose, 10);
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), subscriber_gps, subscriber_path);  
    sync.registerCallback(boost::bind(&multi_callback, _1, _2));
    
    std::cout << "hahah" << std::endl;

    ros::spin();
    return 0;
}
