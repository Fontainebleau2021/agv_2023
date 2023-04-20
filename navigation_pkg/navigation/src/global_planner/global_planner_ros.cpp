#include "global_planner_ros.h"

namespace navigation
{
    void GlobalPlannerRos::Init()
    {
        
        std::string global_path_topic, cur_pose_topic, target_pose_topic;
        nh_.param<std::string>("global_path_topic", global_path_topic, "global_path");
        nh_.param<std::string>("cur_pose_topic", cur_pose_topic, "/odom");
        nh_.param<std::string>("target_pose_topic", target_pose_topic, "/goal");
        global_path_pub_ = nh_.advertise<nav_msgs::Path>(global_path_topic, 10);
        //search_gridcell_pub_ = nh_.advertise<nav_msgs::GridCells>("search_nodes", 10);
        cur_pose_sub_ = nh_.subscribe(cur_pose_topic, 1, &GlobalPlannerRos::CurPoseCallBack, this);
        target_pose_sub_ = nh_.subscribe(target_pose_topic, 1, &GlobalPlannerRos::TargetPoseCallBack, this);
        double s_x, s_y, e_x, e_y;
        nh_.param("start_x", s_x, 0.0);
        nh_.param("start_y", s_y, 0.0);
        nh_.param("end_x", e_x, 10.0);
        nh_.param("end_y", e_y, 10.0);
        cur_pose_.pose.position.x = s_x;
        cur_pose_.pose.position.y = s_y;
        target_pose_.pose.position.x = e_x;
        target_pose_.pose.position.y = e_y;
        global_path_.header.frame_id = "map";
        costmap_ros_->Visualize();
       
    }

    void GlobalPlannerRos::ToNavPathMsg()
    {
        std::vector<std::vector<Grid2D>> search_nodes  = global_planner_->SearchGrid();
        costmap_ros_->PublisheSearchGrid(search_nodes);
        std::vector<Pose2D> global_path;
        global_planner_->GeneratePath(global_path);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.z = 0;
        global_path_.poses.clear();
        global_path_.poses.push_back(cur_pose_);
        for (int i = 1; i < global_path.size() - 1; i++)
        {
            // costmap_2d->MapToWorld(global_path[i].x, global_path[i].y, pose_stamped.pose.position.x, pose_stamped.pose.position.y);
            pose_stamped.pose.position.x = global_path[i].x;
            pose_stamped.pose.position.y = global_path[i].y;
            global_path_.poses.push_back(pose_stamped);
        }
        static int seq = 1;
        global_path_.poses.push_back(target_pose_);
        global_path_.header.stamp = ros::Time::now();
        global_path_.header.seq = seq++;
        global_path_pub_.publish(global_path_);
        ros::Duration(2).sleep();
    }

    bool GlobalPlannerRos::Plan()
    {
        //std::vector<grid
        //ROS_INFO("111");
        costmap_ros_->Visualize();
        std::chrono::steady_clock::time_point s_time = std::chrono::steady_clock::now();
        Grid2D start, end;
        start.theta = 0;
        end.theta = 0;
        auto costmap_2d = costmap_ros_->Costmap2d();
        if (!costmap_2d->WorldToMap(cur_pose_.pose.position.x, cur_pose_.pose.position.y, start.x, start.y))
        {
            ROS_INFO("THE cur pose is invalid");
            return false;
        }
        if (costmap_2d->GridCost(start.x, start.y) != GridType::FREE)
        {
            ROS_INFO("THE cur pose is occupied, maybe location is wrong");
            return false;
        }
        if (!costmap_2d->WorldToMap(target_pose_.pose.position.x, target_pose_.pose.position.y, end.x, end.y))
        {
            ROS_INFO("THE target pose is invalid");
            return false;
        }
        if (costmap_2d->GridCost(end.x, end.y) != GridType::FREE)
        {
            ROS_INFO("THE target pose is occupied");
            return false;
        }
        global_planner_->SetCostmap2d(costmap_2d);
        if (!global_planner_->Plan(start, end))
        {
            ROS_INFO("global plan failed");
            return false;
        }
        auto e_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = e_time - s_time;
        std::cout << "time: " << elapsed.count() << "s" << std::endl;   
        //ROS_INFO("global plan success");
        ToNavPathMsg();
        return true;
    }

    void GlobalPlannerRos::CurPoseCallBack(const nav_msgs::Odometry &odom_msg)
    {
        // ROS_INFO("get cur pose");
        // tf::trans()
        geometry_msgs::PoseStamped odom_pose;
        odom_pose.header = odom_msg.header;
        odom_pose.pose = odom_msg.pose.pose;
        tf_.transform(odom_pose, cur_pose_, "map");
    }

    void GlobalPlannerRos::TargetPoseCallBack(const geometry_msgs::PoseStamped &pose_msg)
    {
        target_pose_ = pose_msg;
    }

} // namespace navigation
