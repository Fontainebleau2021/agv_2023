#include "seed_line_pose.h"

void SeedLinePose(int num_step, int num_ridge, geometry_msgs::Pose2D& start_pose, geometry_msgs::Pose2D& end_pose)
{
    double base_x = (num_step-1) * 12+0.78;
    start_pose.x = base_x + (num_ridge-1)*1.16;
    end_pose.x = start_pose.x;
    if(num_ridge<=5 && num_ridge>=1)
    {
        start_pose.y = 78.4;
        end_pose.y = 2.4;
        start_pose.theta = end_pose.theta = M_PI_2;

    }
    else if(num_ridge<=10 && num_ridge>=6)
    {
        start_pose.y = 2.4;
        end_pose.y = 78.4;
        start_pose.theta = end_pose.theta = -M_PI_2;
    }
    else
    {
        return;
    }
}