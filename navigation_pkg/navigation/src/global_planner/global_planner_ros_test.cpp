
#include "a_star.h"
#include "hybrid_a_star.h"
#include <iostream>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <nav_msgs/Path.h>
#include "ros_api.h"

using namespace navigation;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_ros");
    ros::NodeHandle nh("~"); //私有名称以节点的名字作为命名空间

    RosPublisher<nav_msgs::Path> path_pub(&nh, "global_path");
    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    std::string map_path = "/home/agv/planning_ws/src/navigation/map/grid_map.png";
    cv::Mat colormap = cv::imread(map_path, cv::IMREAD_COLOR);
    std::cout << "图像宽为：" << colormap.cols << "\t高度为：" << colormap.rows << "\t通道数为：" << colormap.channels() << std::endl;
    cv::imshow("temp", colormap);
    cv::waitKey(0);
    cv::Mat cvmap;
    cv::cvtColor(colormap, cvmap, cv::COLOR_BGR2GRAY);
    int row = cvmap.rows, col = cvmap.cols;
    Eigen::MatrixXi b(row, col);
    cv::cv2eigen(cvmap, b);
    Costmap2DPtr costmap_2d = std::make_shared<Costmap2D>(col, row, 0.1, 0.06, 0, 0);
    for (int x = 0; x < col; x++)
    {
        for (int y = 0; y < row; y++)
        {
            // unsigned char cost = static_cast<unsigned char>(b(y, x));
            costmap_2d->SetGridCost(x, y, b(y, x));
        }
    }
    costmap_2d->Inflation();
    Pose2D start(4.5, 30);
    Pose2D end(4, 90);
    GlobalPlanner *planner = new Astar();
    std::chrono::steady_clock::time_point s_time = std::chrono::steady_clock::now();
    int sx, sy, ex, ey;
    if (planner->Plan(start,end,costmap_2d))
    {
        auto e_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = e_time - s_time;
        std::cout << "time: " << elapsed.count() << "s" << std::endl;
        std::vector<Grid2D> path;
        //planner->GetGlobalPath(path);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.z = 0;
        for(int i = 0; i<path.size();i++)
        {

            costmap_2d->MapToWorld(path[i].x, path[i].y, pose_stamped.pose.position.x, pose_stamped.pose.position.y);
            msg.poses.push_back(pose_stamped);
        }
        path_pub.Publish(msg);
        
    }
    else
    {
        std::cout << "plan failed" << std::endl;
    }

    return 0;
}