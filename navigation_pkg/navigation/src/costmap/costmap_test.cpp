#include "costmap_convert.h"
#include "costmap.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>

using namespace navigation;

int main()
{

    std::string map_path = "/home/agv/planning_ws/src/navigation/map/simulation_map.png";
    cv::Mat colormap = cv::imread(map_path, cv::IMREAD_COLOR);
    std::cout << "图像宽为：" << colormap.cols << "\t高度为：" << colormap.rows << "\t通道数为：" << colormap.channels() << std::endl;
    cv::imshow("temp", colormap);
    cv::waitKey(0);
    cv::Mat cvmap;
    cv::cvtColor(colormap, cvmap, cv::COLOR_BGR2GRAY);
    int row = cvmap.rows, col = cvmap.cols;
    Eigen::MatrixXi b(row, col);
    cv::cv2eigen(cvmap, b);
    Costmap2DPtr costmap_2d = std::make_shared<Costmap2D>(col, row, 0.05, 0.05, 0, 0);
    for (int x = 0; x < col; x++)
    {
        for (int y = 0; y < row; y++)
        {
            // unsigned char cost = static_cast<unsigned char>(b(y, x));
            costmap_2d->SetGridCost(x, y, b(y, x));
        }
    }

    CostmapConvert* costmap_convert = new CostmapConvert();
    costmap_convert->updateCostmap2D(costmap_2d);
    costmap_convert->compute();
    std::vector<ObstaclePtr> obst_vector;
    costmap_convert->UpdateObstaclePtr(obst_vector);
    std::cout << costmap_convert->Polygons()->size() << std::endl;
    for (int i = 0; i < costmap_convert->Polygons()->size(); i++)
    {
        //std::cout<<costmap_convert->Polygons()->at(i).points.size();
        //Grid2D c((int)costmap_convert->Polygons()->at(i).points[0].x/1, (int)costmap_convert->Polygons()->at(i).points[0].y/1);
        for (int j = 0; j < costmap_convert->Polygons()->at(i).points.size(); j++)
        {
            // Grid2D n((int)costmap_convert->Polygons()->at(i).points[j].x /1, (int)costmap_convert->Polygons()->at(i).points[j].y/1);
            // cv::line(colormap, cv::Point(c.x, c.y), cv::Point(n.x, n.y), cv::Scalar(100, 0, 0), 2, cv::LINE_8);
            // c = n;
            std::cout<<costmap_convert->Polygons()->at(i).points[j].x<<" "<< costmap_convert->Polygons()->at(i).points[j].y<<" ";
        }
        
        // cv::namedWindow("map", 0);
        // cv::imshow("map", colormap);
        // cv::waitKey(0);
    }
    return 0;


}