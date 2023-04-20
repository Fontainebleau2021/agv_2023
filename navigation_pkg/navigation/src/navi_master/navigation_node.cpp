#include "navigation_ros.h"
#include <tf2_ros/transform_listener.h>

using namespace navigation;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh("~"); //私有名称以节点的名字作为命名空间
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    int control_rate;
    nh.param("control_rate", control_rate, 5);
    ros::Rate loop_rate(control_rate);
    
    NaviMasterRos navi_master_ros("navigation", buffer);

    while (ros::ok())
    {
        navi_master_ros.Run();
        // local_planner_->plan(PoseSE2(0, 0, 0), PoseSE2(3, 2, 0));
        // local_planner_->visualize();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}