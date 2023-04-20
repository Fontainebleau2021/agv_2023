#include "costmap_ros.h"
#include <tf2_ros/transform_listener.h>

using namespace navigation;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "costmap_ros");
    //ros::NodeHandle nh("~"); //私有名称以节点的名字作为命名空间
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    CostmapRos costmap_ros("costmap",buffer);
    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        costmap_ros.Visualize();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}