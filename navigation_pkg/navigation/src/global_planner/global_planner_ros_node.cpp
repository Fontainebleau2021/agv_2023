#include "global_planner_ros.h"
#include <tf2_ros/transform_listener.h>

using namespace navigation;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global");
    //ros::NodeHandle nh("~"); //私有名称以节点的名字作为命名空间
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    GlobalPlannerRos global_planner_ros("global_planner", buffer);
    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        global_planner_ros.Plan();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}