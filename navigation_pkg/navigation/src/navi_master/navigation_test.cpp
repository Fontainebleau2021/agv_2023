
#include <iostream>
#include <chrono>
#include "navigation.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
// (20,30) (40,50) (40, 30) (20,50)

using namespace navigation;

int main(int argc, char** argv)
{
    
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
    Pose2D start(3.5, 3,-M_PI/4);
    Pose2D end(4, 90,-M_PI/4);

    GlobalPlannerPtr g_planner = std::make_shared<Astar>(costmap_2d);
    std::vector<ObstaclePtr> obst_vector;
    int x1 = 50,x2 = 770, y = 210;
    for(int i =1;i<=9;i++)
    {
        int y1 = y + 200*i;
        obst_vector.emplace_back(std::make_shared<LineObstacle>(x1*0.1,y1*0.06,x2*0.1,y1*0.06));
    }


    obst_vector.emplace_back(std::make_shared<LineObstacle>(10*0.1,210*0.06,18*0.1,210*0.06));
    obst_vector.emplace_back(std::make_shared<LineObstacle>(36*0.1,210*0.06,90*0.1,210*0.06));
    obst_vector.emplace_back(std::make_shared<LineObstacle>(730*0.1,2210*0.06,770*0.1,2210*0.06));
    obst_vector.emplace_back(std::make_shared<LineObstacle>(802*0.1,2210*0.06,810*0.1,2210*0.06));

    TebConfig config;
    // PoseSE2 start(4, 5, -M_PI/4);
    // PoseSE2 end(4, 90, -M_PI/4);
    
    ViaPointContainer via_points;
    // Setup robot shape model
    Eigen::Vector2d rec1(10*0.1, 10*0.06), rec2(10*0.1, 2210*0.06), rec3(730*0.1, 2210*0.06), 
    rec4(730*0.1, 2410*0.06), rec5(810*0.1,2410*0.06), rec6(810*0.1,210*0.06), rec7(90*0.1,210*0.06),rec8(90*0.1,10*0.06);

    PolygonCon ver;
    ver.push_back(rec1);
    ver.push_back(rec2);
    ver.push_back(rec3);
    ver.push_back(rec4);
    ver.push_back(rec5);
    ver.push_back(rec6);
    ver.push_back(rec7);
    ver.push_back(rec8);
    auto po_obs = std::make_shared<PolygonObstacle>(ver);
    po_obs->FinalizePolygon();
    obst_vector.push_back(po_obs);
    RobotFootprintModelPtr robot_model = std::make_shared<CircularRobotFootprint>(1.0);
    LocalPlannerPtr l_planner = std::make_shared<HomotopyClassPlanner>(config, &obst_vector, robot_model, &via_points);
    //auto planner = new HomotopyClassPlanner(config, &obst_vector, robot_model, &via_points);
    NaviMaster navi(g_planner, l_planner);
    navi.SetCurPose(start);
    navi.SetTargetPose(end);
    bool success = navi.Plan();
    if(success)
    {
        
    }

    cv::imshow("map", colormap);
    cv::waitKey(0);
    return 0;
}