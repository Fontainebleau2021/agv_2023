#ifndef GLOBAL_PLANNER_ROS_H
#define GLOBAL_PLANNER_ROS_H

#include "global_planner.h"
#include "a_star.h"
#include "LPAstar.h"
#include "d_star_lite.h"
#include "hybrid_a_star.h"
#include "dijkstra.h"
#include "costmap_ros.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <chrono>
namespace navigation
{
    class GlobalPlannerRos
    {
    public:
        GlobalPlannerRos() = default;
        GlobalPlannerRos(std::string name, tf2_ros::Buffer &tf) : tf_(tf)
        {
            nh_ = ros::NodeHandle("~/" + name);
            costmap_ros_ = std::make_shared<CostmapRos>("costmap", tf);
            Init();
            bool enable_astar, enable_dijkstra, enable_dstar;
            nh_.param("enable_astar", enable_astar, true);
            nh_.param("enable_dijkstra", enable_dijkstra, false);
            nh_.param("enable_dstar", enable_dstar, true);
            if (enable_astar)
            {
                global_planner_ = std::make_shared<Astar>();
            }
            else if (enable_dijkstra)
            {
                global_planner_ = std::make_shared<Dijkstra>();
            }
            else if(enable_dstar)
            {
                global_planner_ = std::make_shared<DStarLite>();
            }

            
            // costmap_ros_->Visualize();
            // auto costmap_2d = costmap_ros_->Costmap2d();
            // global_planner_->SetCostmap2d(costmap_2d);
            // global_planner_->Init();
        }
        ~GlobalPlannerRos() {}

        void Init();

        void ToNavPathMsg();
        bool Plan();

        void CurPoseCallBack(const nav_msgs::Odometry &odom_msg);
        void TargetPoseCallBack(const geometry_msgs::PoseStamped &odom_msg);

    private:
        ros::NodeHandle nh_;
        tf2_ros::Buffer &tf_;
        CostmapRosPtr costmap_ros_;

        geometry_msgs::PoseStamped cur_pose_;
        geometry_msgs::PoseStamped target_pose_;

        GlobalPlannerPtr global_planner_;
        nav_msgs::Path global_path_;
        ros::Publisher global_path_pub_;

        

        ros::Subscriber target_pose_sub_;
        ros::Subscriber cur_pose_sub_;
        std::vector<geometry_msgs::Point32> points_;
    };
} // namespace navigation

#endif // GLOBAL_PLANNER_ROS_H