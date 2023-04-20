#ifndef SEED_LINE_POSE_H
#define SEED_LINE_POSE_H

#include <geometry_msgs/Pose2D.h>
//得到在第几跨第几垄的起点和终点坐标以及车的朝向
// 1 2 3 4 5 是朝Y负方向 6 7 8 9 10 是朝Y正方向
void SeedLinePose(int num_step, int num_ridge, geometry_msgs::Pose2D& start_pose, geometry_msgs::Pose2D& end_pose);

#endif // SEED_LINE_POSE_H