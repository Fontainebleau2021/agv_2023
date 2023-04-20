#include "costmap_ros.h"
#include <opencv2/opencv.hpp>

namespace navigation
{


    void CostmapRos::ParamterLoad()
    {
        std::string map_path, obstacle_topic;
        double resolution_x, resolution_y, origin_x, origin_y, inflation_radius;

        nh_.param("resolution_x", resolution_x, 0.1);
        nh_.param("resolution_y", resolution_y, 0.06);
        nh_.param("origin_x", origin_x, 0.0);
        nh_.param("origin_y", origin_y, 0.0);
        nh_.param("inflation_radius", inflation_radius, 0.8);
        nh_.param<std::string>("frame_id", frame_id_, "map");
        nh_.param<std::string>("obstacle_topic", obstacle_topic, "/points");
        map_update_sub_ = nh_.subscribe(obstacle_topic, 1, &CostmapRos::UpdateMapCallBack, this);
        
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("map_path", map_path, "/home/agv/planning_ws/src/navigation/map/grid_map.png");
        cv::Mat cvmap = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
        static_costmap_ = new unsigned char[cvmap.cols * cvmap.rows];
        navi_costmap_2d_ = std::make_shared<Costmap2D>(cvmap.cols, cvmap.rows, resolution_x, resolution_y, origin_x, origin_y, inflation_radius);
        for (int x = 0; x < cvmap.cols; x++)
        {
            for (int y = 0; y < cvmap.rows; y++)
            {
                // int a_y = cvmap.rows - 1 - y;
                navi_costmap_2d_->SetGridCost(x, y, cvmap.at<uchar>(y, x));
                static_costmap_[navi_costmap_2d_->getIndex(x, y)] = cvmap.at<uchar>(y, x);
                // copy.at<uchar>(a_y,x) = cvmap.at<uchar>(y, x);
            }
        }
;
        map_grid_cells_.header.frame_id = frame_id_;
        map_inflation_grid_cells_.header.frame_id = frame_id_;
        
        navi_costmap_2d_->Inflation();
        map_grid_cells_.cell_height = navi_costmap_2d_->ResolutionY();
        map_grid_cells_.cell_width = navi_costmap_2d_->ResolutionX();
        map_inflation_grid_cells_.cell_height = navi_costmap_2d_->ResolutionY();
        map_inflation_grid_cells_.cell_width = navi_costmap_2d_->ResolutionX();
        ToGridCellsMsg();
    }

    void CostmapRos::ToGridCellsMsg()
    {
        geometry_msgs::Point obs_point;
        geometry_msgs::Point inflation_point;
        map_grid_cells_.cells.clear();
        map_inflation_grid_cells_.cells.clear();
        for (int i = 0; i < navi_costmap_2d_->SizeX(); i++)
        {
            for (int j = 0; j < navi_costmap_2d_->SizeY(); j++)
            {
                if (navi_costmap_2d_->GridCost(i, j) == GridType::OBSTACLE)
                {
                    navi_costmap_2d_->MapToWorld(i, j, obs_point.x, obs_point.y);
                    obs_point.z = 0;
                    map_grid_cells_.cells.push_back(obs_point);
                }
                else if (navi_costmap_2d_->GridCost(i, j) == GridType::INFLATION)
                {
                    navi_costmap_2d_->MapToWorld(i, j, inflation_point.x, inflation_point.y);
                    inflation_point.z = 0;
                    map_inflation_grid_cells_.cells.push_back(inflation_point);
                }
            }
        }

    }

    void CostmapRos::Visualize()
    {
        static int seq = 1;
        map_inflation_grid_cells_.header.stamp = ros::Time::now();
        map_inflation_grid_cells_.header.seq = seq++;
        map_grid_cells_.header.stamp = ros::Time::now();
        map_grid_cells_.header.seq = map_inflation_grid_cells_.header.seq;
        map_grid_publisher_.publish(map_grid_cells_);
        map_inflation_grid_publisher_.publish(map_inflation_grid_cells_);
    }

    void CostmapRos::CostmapUpdate()
    {
        memcpy(navi_costmap_2d_->Costmap(), static_costmap_, navi_costmap_2d_->SizeX() * navi_costmap_2d_->SizeY() * sizeof(unsigned char));
        navi_costmap_2d_->Inflation();
        //
        int g_x, g_y;
        for (int i = 0; i < points_.size(); i++)
        {
            if (navi_costmap_2d_->WorldToMap(points_[i].x, points_[i].y, g_x, g_y))
            {
                navi_costmap_2d_->SetGridCost(g_x, g_y, GridType::OBSTACLE);
                navi_costmap_2d_->InflationCell(g_x, g_y);
            }
        }
        ToGridCellsMsg();
    }

} // namespace navigation
