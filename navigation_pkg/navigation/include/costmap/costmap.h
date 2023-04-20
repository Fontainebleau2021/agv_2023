
#ifndef COSTMAP_2D_COSTMAP_2D_H_
#define COSTMAP_2D_COSTMAP_2D_H_

#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>
#include <stdio.h>
#include <memory>
#include <geometry_msgs/Point.h>

/*
说明：规划地图的栅格代价地图类
功能：
1：记录规划地图的像素尺寸大小 size_x_; size_y_;  地图左上角 原点在世界坐标系中的坐标  地图的分辨率  机器人的膨胀半径  自由区域值：0  障碍物区域：255
2： 用char数组来记录栅格状态
*/
namespace navigation
{

    struct MapLocation
    {
        int x;
        int y;
    };

    class Costmap2D
    {

    public:
        // 构造函数和析构函数
        Costmap2D(int cells_size_x, int cells_size_y, double x_resolution, double y_resolution,
                  double origin_x, double origin_y, double inflation_radius = 0.5, unsigned char default_value = 0);

        Costmap2D(const Costmap2D &map);

        Costmap2D &operator=(const Costmap2D &map);

        Costmap2D();

        virtual ~Costmap2D();
        // 栅格数组初始化
        void InitCostmap(int size_x, int size_y);
        // 栅格数组重置为default_value_
        void ResetCostmap();

        // 重新设置栅格地图数组与分辨率等参数
        void ResizeCostmap(int size_x, int size_y, double x_resolution,
                           double y_resolution, double origin_x, double origin_y);
        // 栅格数组删除
        void DeleteCostmap();

        // 返回栅格状态
        unsigned char GridCost(int mx, int my) const;
        // 设置栅格状态
        void SetGridCost(int mx, int my, unsigned char cost);

        // 地图像素坐标转换为世界坐标
        void MapToWorld(int mx, int my, double &wx, double &wy) const;

        // 世界坐标转换为栅格像素坐标
        bool WorldToMap(double wx, double wy, int &mx, int &my) const;

        // 根据像素坐标得到数组索引
        inline int getIndex(int mx, int my) const
        {
            return my * size_x_ + mx;
        }
        // 索引转换为像素坐标
        inline void indexToCells(int index, int &mx, int &my) const
        {
            my = index / size_x_;
            mx = index - (my * size_x_);
        }

        // 得到地图的信息
        int SizeX() { return size_x_; }
        int SizeY() { return size_y_; }

        double getSizeInMetersX() const { return (size_x_ - 1 + 0.5) * x_resolution_; }

        double getSizeInMetersY() const { return (size_y_ - 1 + 0.5) * y_resolution_; }
        double OriginY() { return origin_y_; }
        double OriginX() { return origin_x_; }
        double ResolutionX() { return x_resolution_; }
        double ResolutionY() { return y_resolution_; }
        double InflationRadius() { return inflation_radius_; }
        unsigned char *Costmap() { return costmap_; }
        // 判断像素索引是否有效
        bool Valid(int mx, int my)
        {
            if (mx < 0 || mx >= size_x_ || my < 0 || my >= size_y_)
            {
                return false;
            }
            return true;
            // if(GridCost(mx, my)== GridType::OBSTACLE)
        }

        // 对地图进行膨胀
        void InflationCell(int cell_x, int cell_y);
        void Inflation();

        // 用来判断轨迹坐标
        double footprintCost(double x, double y, double theta, const std::vector<geometry_msgs::Point> &footprint_spec, double inscribed_radius = 0.0, double circumscribed_radius = 0.0)
        {
            return 1;
        }

        
        
        

        // void Copy()
    private:
        int size_x_;
        int size_y_;
        double x_resolution_;
        double y_resolution_;
        double origin_x_;
        double origin_y_;
        unsigned char *costmap_;
        unsigned char default_value_;

        double inflation_radius_;
        int inflation_cell_size_x_;
        int inflation_cell_size_y_;
    };

    typedef std::shared_ptr<Costmap2D> Costmap2DPtr;
} // namespace navigation

#endif // COSTMAP_2D_COSTMAP_2D_H
