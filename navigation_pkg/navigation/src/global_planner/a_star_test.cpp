#include "a_star.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>


int main()
{
   
    cv::Mat colormap = cv::imread("map.png",cv::ImreadModes::IMREAD_COLOR);
    cv::Mat cvmap = cv::imread("map.png",cv::ImreadModes::IMREAD_GRAYSCALE);
    //cv::cvtColor(image,cvmap,cv::COLOR_BGR2GRAY);
    std::cout << "图像宽为：" << cvmap.cols << "\t高度为：" << cvmap.rows << "\t通道数为：" << cvmap.channels() << std::endl;
    int row = cvmap.rows, col = cvmap.cols;
    Eigen::MatrixXi b(row, col);
    cv::cv2eigen(cvmap, b);
    Eigen::ArrayXXi costmap = b.array();
    Eigen::RowVector2i start(0,0);
    Eigen::RowVector2i end(899,899);
    planner::GraphSearchPlanner* global_planner =  new planner::Dijkstra(start, end, costmap, 0, 0, 0.3);
    bool secc = global_planner->Plan();
    //global_planner->GeneratePath();
    //global_planner->OutPut();
    std::vector<Eigen::RowVector2i> path = global_planner->Path();
    for (int i =1; i<path.size(); i++)
    {
        cv::line(colormap, cv::Point(start.x(), start.y()), cv::Point(path[i].x(), path[i].y()), cv::Scalar(100,100,0), 2 ,cv::LINE_8);
        start = path[i];
    }
    cv::imshow("map", colormap);
    cv::waitKey(0);
    // auto path = dijtest.Path();
    // std::vector<std::vector<int>> ans;
  
    // cv::drawContours(image, ans,-1, cv::Scalar::all(100));
    // cv::imshow("ans", image);
    // cv::waitKey(0);
    return 0;
}