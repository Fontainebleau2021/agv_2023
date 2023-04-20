// #include "dijkstra.h"
#include "a_star.h"
//#include "d_star_lite.h"
// #include "LPAstar.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
int main()
{

    // int size_x = 5;
    // int size_y = 5;
    // Eigen::ArrayXXi map(size_y, size_x);

    // for (int y = 0; y < size_y; y++)
    // {
    //     for (int x = 0; x < size_x; x++)
    //     {
    //         map(y, x) = 255;
    //     }
    // }
    // Eigen::ArrayXXi obs(3, 3);
    // obs << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    // map.block(1, 1, 3, 3) = obs;
    // std::cout << map << std::endl;
    //map.block(8, 8, 3, 3) = obs;

    cv::Mat colormap = cv::imread("map.png",cv::ImreadModes::IMREAD_COLOR);
    cv::Mat cvmap;
    cv::cvtColor(colormap, cvmap, cv::COLOR_BGR2GRAY);
    std::cout << "图像宽为：" << cvmap.cols << "\t高度为：" << cvmap.rows << "\t通道数为：" << cvmap.channels() << std::endl;
    int row = cvmap.rows, col = cvmap.cols;
    Eigen::MatrixXi b(row, col);
    cv::cv2eigen(cvmap, b);
    Eigen::ArrayXXi costmap = b.array();
    Eigen::RowVector2i start(0, 0);
    Eigen::RowVector2i end(499, 499);
    planner::GraphSearchPlanner *global_planner = new planner::Astar(start, end, costmap, 0, 0, 0.3);
    auto s_time = std::chrono::steady_clock::now();
    bool secc = global_planner->Plan();
    auto e_time = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::micro> elapsed = e_time - s_time;
    std::cout<< "time: "  << elapsed.count() << "us" << std::endl;
    std::vector<Eigen::RowVector2i> path = global_planner->Path();
    for (int i = 1; i < path.size(); i++)
    {
        cv::line(colormap, cv::Point(start.x(), start.y()), cv::Point(path[i].x(), path[i].y()), cv::Scalar(100,100,0), 2 ,cv::LINE_8);
        start = path[i];
        //std::cout << path[i] << std::endl;
    }
    cv::imshow("map", colormap);
    cv::waitKey(0);
    return 0;
}