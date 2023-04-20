#ifndef NAVIGATION_ROS_H
#define NAVIGATION_ROS_H

#include "navi_state.h"
#include "costmap_ros.h"
#include "global_planner.h"
#include "a_star.h"

#include "homotopy_class_planner.h"
#include "planner_interface.h"

#include "seed_control.h"
#include "seed_line_pose.h"

#include <chrono>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <dynamic_reconfigure/server.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace navigation
{
    const double seed_y_max = 10.0;
    const double seed_y_min = 5.0;
    const double seed_x_start = 0.78;
    const double seed_width = 1.0;
    const double seed_gap = 0.16;
    const double seed_center_dis = seed_width + seed_gap;
    const double RotateTargetTheta[] = {-0.2, M_PI_2, M_PI, -M_PI_2};
    const double seedlineTheta[] = { M_PI, M_PI_2, 0, -M_PI_2};
    const double seedRotateTheta[] = { M_PI, M_PI_2, 0, -M_PI_2};
    const double seed_LineXYTerminal[11][2] = {
        seed_y_min, seed_x_start+1*seed_center_dis,
        seed_y_max, seed_x_start+5*seed_center_dis,
        seed_y_min, seed_x_start+1*seed_center_dis,
        seed_y_max, seed_x_start+6*seed_center_dis,
        seed_y_min, seed_x_start+2*seed_center_dis,
        seed_y_max, seed_x_start+7*seed_center_dis,
        seed_y_min, seed_x_start+3*seed_center_dis,
        seed_y_max, seed_x_start+8*seed_center_dis,
        seed_y_min, seed_x_start+4*seed_center_dis,
        seed_y_max, seed_x_start+8*seed_center_dis,
        seed_y_min, seed_x_start+8*seed_center_dis,
    };
    

    enum VisualizeType
    {
        COSTMAP = 1,
        PLAN = 2,
    };

    enum NaviCommand
    {
        WaitCmd = 1,
        NaviPlanCmd = 2,
        NaviControlCmd,
        ReInitCmd,
        StopCmd,
        LineControlCmd,
        RotateControlCmd,
        StratCmd,
        TurnCmd,
        PIDseed,
    };

    class NaviMasterRos
    {
    public:
        NaviMasterRos(std::string name, tf2_ros::Buffer &tf);
        ~NaviMasterRos() {}

        // 主运行函数 进行状态执行和状态转换
        void Run();

        // 设置机器人当前导航状态
        void SetNaviState(const NaviStatePtr &next_state)
        {
            cur_state_ = next_state;
        }
        // 参数等初始化
        void Init();

        // 手动写入local_planner需要的障碍物信息 要根据实际的地图进行修改
        void LocalMapObsUpdate();

        // 全局规划
        bool GlobalPlan();

        // 将规划结果转换成ROS消息类型
        void ToNavPathMsg();

        // 局部规划 根据全局路径结果进行规划
        bool LocalInitPlan();

        // 局部规划 只采用当前和目标位置信息
        bool LocalPlan();

        // local_planner里面计算输出的速度
        void GetControlVelocity();

        // 停止函数 速度发布0
        void Stop();
        //一些指令的订阅 包括当前位置 目标位置 rviz仿真点选取 和导航的指令
        void CurPoseCallBack(const nav_msgs::Odometry &odom_msg);
        void TargetPoseCallBack(const geometry_msgs::PoseStamped &odom_msg);
        void RvizClickPointCallBack(const geometry_msgs::PointStamped &point);
        void NaviCmdCallBack(const std_msgs::Int8 &cmd);
        
        //判断是否到达目标位置 误差在config里面设置
        bool IsNaviArriveTarget();
        bool IsArriveTarget(geometry_msgs::Pose &target_pose, double xy_tolerant, double theta_tolerant);
        //可视化函数
        void Visualize(VisualizeType visulize_type);

        NaviCommand NaviCmd() { return navi_cmd_; }
        void SetNaviCmd(NaviCommand cmd);

        // 和PID控制相关的函数
        //直线行驶控制输出计算，需要根据方向设定直线的倾角heading 通过SeedPIDControl：：SetHeading
        void LineControl(double heading);
        //原地旋转控制输出
        void RotateControl(double target_theta);
        //发布导航目的地
        void PubPIDGoal(double x, double y, double theta);
        //可视化
        void SeedLineVisualize();
        //每一次进行直线控制前，需要更新当前位置为起始位置，从而进行控制计算 在navi_state.cpp StateChange函数中用到
        void UpdateLineStartPose()
        {
            if (!in_control_)
            {
                pid_control_->SetStartPose(cur_pose_.pose);
            }
        }
        // 判断是否到达 输入两个参数 一个是垄的索引 一个是朝向
        bool SeedIsLineArriveTerminal(int dir, int idx);
        bool SeedIsRotateArriveTerminal(int idx);
        bool IsLineArriveTerminal(int dir, int idx);
        bool IsRotateArriveTerminal(int idx);
        bool in_control_ = false;
        bool in_seed_ = false;
        bool seed_run = false;

        void CB_reconfigure(TebLocalPlannerReconfigureConfig &reconfig, uint32_t level)
        {
            config_.reconfigure(reconfig);
        }

    private:
        ros::NodeHandle nh_;
        NaviStatePtr cur_state_;
        NaviCommand navi_cmd_;
        GlobalPlannerPtr global_planner_;
        PlannerInterfacePtr local_planner_;

        geometry_msgs::PoseStamped cur_pose_;
        geometry_msgs::PoseStamped target_pose_;

        // tf translation
        tf2_ros::Buffer &tf_;
        // costmap
        CostmapRosPtr costmap_2d_ros_;
        Costmap2DPtr costmap_2d_;

        //  ros topic pub
        nav_msgs::Path global_path_;
        ros::Publisher global_path_pub_;
        ros::Publisher cmd_vel_pub_;
        ros::Publisher bunker_cmd_vel_pub_;
        ros::Publisher cur_pose_pub_;
        ros::Publisher state_pub_;
        ros::Publisher trans_shift_pub_;
        ros::Publisher lio_state_sub_;
        ros::Publisher publish_PID_goal;
        
        geometry_msgs::Twist cur_vel_;

        // local planner settings
        TebConfig config_;
        TebVisualizationPtr visual_;
        RobotFootprintModelPtr robot_model_;
        ViaPointContainer via_points_;
        boost::shared_ptr<dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>> dynamic_recfg_;
        dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb_;
        // suscriber
        ros::Subscriber target_pose_sub_;
        ros::Subscriber cur_pose_sub_;
        ros::Subscriber rviz_point_sub_;
        ros::Subscriber navi_cmd_sub_;

        std::vector<ObstaclePtr> obst_vector_;

        SeedPIDControlPtr pid_control_;
        ros::Publisher line_pose_pub_;
        ros::Publisher line_dir_pub_;
        double linecontrol_xy_tolerance_;
        double rotate_theta_tolerance_;
    };

} // namespace navigation

#endif // NAVIGATION_ROS_H