#ifndef NAVI_COSTMAP_ROS_H
#define NAVI_COSTMAP_ROS_H

#include "costmap.h"
#include "grid.h"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>

// #include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

/*
说明：结合ROS的规划代价地图类

功能：
1：读取代价地图参数，初始化成员
2：订阅传感器信息，更新代价地图
3：代价地图rviz可视化
4（未完成）：代价地图与多边形障碍物表示的转换——用于local_planner

*/

namespace navigation
{

    class CostmapRos
    {

    public:
        CostmapRos(std::string name, tf2_ros::Buffer &tf) : tf_(tf), update_(false)
        {
            nh_ = ros::NodeHandle("~/" + name);
            ParamterLoad();

            map_grid_publisher_ = nh_.advertise<nav_msgs::GridCells>("grid_cell", 10);
            map_inflation_grid_publisher_ = nh_.advertise<nav_msgs::GridCells>("inflated_grid_cell", 10);
            map_grid_cells_.header.frame_id = frame_id_;
            map_inflation_grid_cells_.header.frame_id = frame_id_;
            navi_costmap_2d_->Inflation();
            map_grid_cells_.cell_height = navi_costmap_2d_->ResolutionY();
            map_grid_cells_.cell_width = navi_costmap_2d_->ResolutionX();
            map_inflation_grid_cells_.cell_height = navi_costmap_2d_->ResolutionY();
            map_inflation_grid_cells_.cell_width = navi_costmap_2d_->ResolutionX();
        }

        ~CostmapRos() {}
        // 地图参数加载
        void ParamterLoad();

        // 栅格数组转换为rviz可视化地图消息数据类型
        void ToGridCellsMsg();
        
        // rviz可视化
        void Visualize();

        // 根据订阅的消息更新代价地图—未完成
        void CostmapUpdate();

        // 订阅传感器消息的回调函数
        void UpdateMapCallBack(const sensor_msgs::PointCloud &point_cloud_msg)
        {
            points_ = point_cloud_msg.points;
            // CostmapUpdate();
            update_ = true;
        }

        // 返回代价地图类对象
        Costmap2DPtr Costmap2d()
        {
            return navi_costmap_2d_;
        }

    private:
        ros::NodeHandle nh_;
        tf2_ros::Buffer &tf_;

        unsigned char *static_costmap_;//存储静态的图片地图信息
        Costmap2DPtr navi_costmap_2d_;

        ros::Publisher map_grid_publisher_;
        ros::Publisher map_inflation_grid_publisher_;
        ros::Subscriber map_update_sub_;

        nav_msgs::GridCells map_grid_cells_;
        nav_msgs::GridCells map_inflation_grid_cells_;

        std::vector<geometry_msgs::Point32> points_;
        bool update_;
        std::string frame_id_;
    };

    using CostmapRosPtr = std::shared_ptr<CostmapRos>;

} // namespace navigation

#endif // NAVI_COSTMAP_ROS_H