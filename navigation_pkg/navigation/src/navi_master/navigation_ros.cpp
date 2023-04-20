#include "navigation_ros.h"
#include <opencv2/opencv.hpp>
#include "visualization.h"

namespace navigation
{

    NaviMasterRos::NaviMasterRos(std::string name, tf2_ros::Buffer &tf) : tf_(tf)
    {
        nh_ = ros::NodeHandle("~/" + name);
        costmap_2d_ros_ = std::make_shared<CostmapRos>("costmap", tf);
        cur_state_ = std::make_shared<InitState>();
    }

    void NaviMasterRos::Run()
    {
        
        cur_state_->Excute(this);
        cur_state_->StateChange(this);
    }

    void NaviMasterRos::Init()
    {

        // ros param init on navigation
        std::string global_path_topic, frame_id, cur_pose_topic, target_pose_topic;
        std::string map_path, vel_topic, navi_cmd_topic;
        nh_.param<std::string>("cur_pose_topic", cur_pose_topic, "/odom");
        nh_.param<std::string>("target_pose_topic", target_pose_topic, "/goal");
        nh_.param<std::string>("frame_id", frame_id, "/map");

        cur_pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/cur_pose", 10);
        global_path_pub_ = nh_.advertise<nav_msgs::Path>("global_path", 10);
        cur_pose_sub_ = nh_.subscribe(cur_pose_topic, 1, &NaviMasterRos::CurPoseCallBack, this);
        target_pose_sub_ = nh_.subscribe(target_pose_topic, 1, &NaviMasterRos::TargetPoseCallBack, this);

        nh_.param<std::string>("navi_cmd_topic", navi_cmd_topic, "/navi_cmd");
        navi_cmd_sub_ = nh_.subscribe(navi_cmd_topic, 1, &NaviMasterRos::NaviCmdCallBack, this);
        //lio_state_sub_ = nh_.subscribe("/lio_state", 1, &NaviMasterRos::liostateCallBack, this);

        nh_.param<std::string>("vel_topic", vel_topic, "/cmd_vel");
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(vel_topic, 10);
        state_pub_ = nh_.advertise<std_msgs::Int8>("/navicmd_state", 10);
        trans_shift_pub_ = nh_.advertise<std_msgs::Int8>("/trans_shift", 10);
        
        nh_.param<std::string>("map_path", map_path, "/home/agv/planning_ws/src/navigation/map/grid_map.png");
        nh_.param("linecontrol_xy_tolerance", linecontrol_xy_tolerance_, 0.1);
        nh_.param("rotate_theta_tolerance", rotate_theta_tolerance_, 0.04);

        // 规划器的初始化
        global_planner_ = std::make_shared<Astar>();
        // 现在是手动把地图里面障碍物写进去
        LocalMapObsUpdate();

        ros::NodeHandle l_n("~teb_local_planner");
        config_.loadRosParamFromNodeHandle(l_n);
        dynamic_recfg_ = boost::make_shared<dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>>(l_n);
        dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb_ = boost::bind(&NaviMasterRos::CB_reconfigure, this, _1, _2);
        dynamic_recfg_->setCallback(cb_);
        visual_ = TebVisualizationPtr(new TebVisualization(l_n, config_));
        robot_model_ = boost::make_shared<CircularRobotFootprint>(0.6);
        local_planner_ = HomotopyClassPlannerPtr(new HomotopyClassPlanner(config_, &obst_vector_, robot_model_, visual_, &via_points_));

        // 回字形路线位置的发布
        line_pose_pub_ = nh_.advertise<visualization_msgs::Marker>("line_marks", 100);
        line_dir_pub_ = nh_.advertise<geometry_msgs::PoseArray>("line_pose", 100);

        bunker_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 10);
        rviz_point_sub_ = nh_.subscribe("/clicked_point", 1, &NaviMasterRos::RvizClickPointCallBack, this);
        global_path_.header.frame_id = frame_id;

        costmap_2d_ = costmap_2d_ros_->Costmap2d();

        //
        pid_control_ = std::make_shared<SeedPIDControl>("seed_control");

        publish_PID_goal = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, true);

        // pid_control_ = new SeedPIDControl(nh_);
    }

    // 这个要根据实际的地图进行修改
    void NaviMasterRos::LocalMapObsUpdate()
    {
        /*
        // 库房障碍物
        // 线
        obst_vector_.push_back(boost::make_shared<LineObstacle>(25 * 0.02, 25 * 0.02, 0.5, 8.5));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(0.5, 8.5, 12.5, 8.5));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(0.5, 0.5, 12.5, 0.5));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(12.5, 0.5, 12.5, 1.4));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(12.5, 3.28, 12.5, 8.5));
        PolygonObstacle *polyobst1 = new PolygonObstacle;
        polyobst1->pushBackVertex(0.5, 9 - 7.4);
        polyobst1->pushBackVertex(0.5, 9 - 8.5);
        polyobst1->pushBackVertex(5.2, 9 - 8.5);
        polyobst1->pushBackVertex(5.2, 9 - 7.4);
        polyobst1->finalizePolygon();
        obst_vector_.emplace_back(polyobst1);

        PolygonObstacle *polyobst2 = new PolygonObstacle;
        polyobst2->pushBackVertex(6.96, 9 - 1.4);
        polyobst2->pushBackVertex(11.46, 9 - 1.4);
        polyobst2->pushBackVertex(11.46, 9 - 2.28);
        polyobst2->pushBackVertex(6.962, 9 - 2.28);
        polyobst2->finalizePolygon();
        obst_vector_.emplace_back(polyobst2);

        PolygonObstacle *polyobst3 = new PolygonObstacle;
        polyobst3->pushBackVertex(8.84, 9 - 7.7);
        polyobst3->pushBackVertex(8.84, 9 - 8.5);
        polyobst3->pushBackVertex(12.5, 9 - 8.5);
        polyobst3->pushBackVertex(12.5, 9 - 7.7);
        polyobst3->finalizePolygon();
        obst_vector_.emplace_back(polyobst3);

        PolygonObstacle *polyobst4 = new PolygonObstacle;
        polyobst4->pushBackVertex(7.76, 9 - 3.68);
        polyobst4->pushBackVertex(7.76, 9 - 2.28);
        polyobst4->pushBackVertex(6.962, 9 - 2.28);
        polyobst4->pushBackVertex(6.962, 9 - 3.68);
        polyobst4->finalizePolygon();
        obst_vector_.emplace_back(polyobst4);
        */

        obst_vector_.push_back(boost::make_shared<LineObstacle>(-12, 6.4, -12, -1.6));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(-12, 6.4, 0, 6.4));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(-0, 1.1, 0, 6.4));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(-12, -1.6, 0, -1.6));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(0, -1.6, 0, -1.1));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(0, -1.6, 108, -1.6));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(0, 6.4, 0, 82.4));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(0, 82.4, 120, 82.4));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(120, 74.4, 120, 82.4));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(120, 74.4, 108, 74.4));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(108, 82.4, 108, 81.9));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(108, 79.7, 108, 74.4));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(108, -1.6, 108, 74.4));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(12, 78.4, 12, 2.4));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(24, 78.4, 24, 2.4));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(36, 78.4, 36, 2.4));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(48, 78.4, 48, 2.4));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(60, 78.4, 60, 2.4));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(72, 78.4, 72, 2.4));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(84, 78.4, 84, 2.4));
        obst_vector_.push_back(boost::make_shared<LineObstacle>(96, 78.4, 96, 2.4));

        // 四边形
        auto polyobst1 = boost::make_shared<PolygonObstacle>();
        polyobst1->pushBackVertex(-12, -0.5);
        polyobst1->pushBackVertex(-7.25, -0.5);
        polyobst1->pushBackVertex(-7.25, -1.6);
        polyobst1->pushBackVertex(-12, -1.6);

        polyobst1->finalizePolygon();
        obst_vector_.emplace_back(polyobst1);
        auto polyobst2 = boost::make_shared<PolygonObstacle>();
        polyobst2->pushBackVertex(-3.65, -0.5);
        polyobst2->pushBackVertex(-0.05, -0.5);
        polyobst2->pushBackVertex(-0.05, -1.6);
        polyobst2->pushBackVertex(-3.65, -1.6);

        polyobst2->finalizePolygon();
        obst_vector_.emplace_back(polyobst2);
        auto polyobst3 = boost::make_shared<PolygonObstacle>();
        polyobst3->pushBackVertex(109.98, 82.4);
        polyobst3->pushBackVertex(110.7, 82.4);
        polyobst3->pushBackVertex(110.7, 81.9);
        polyobst3->pushBackVertex(109.98, 81.9);

        polyobst3->finalizePolygon();
        obst_vector_.emplace_back(polyobst3);

        auto polyobst4 = boost::make_shared<PolygonObstacle>();
        polyobst4->pushBackVertex(112.68, 82.4);
        polyobst4->pushBackVertex(114.48, 82.4);
        polyobst4->pushBackVertex(114.48, 81.6);
        polyobst4->pushBackVertex(112.68, 81.6);

        polyobst4->finalizePolygon();
        obst_vector_.emplace_back(polyobst4);

        auto polyobst5 = boost::make_shared<PolygonObstacle>();
        polyobst5->pushBackVertex(115.37, 82.4);
        polyobst5->pushBackVertex(120, 82.4);
        polyobst5->pushBackVertex(120, 80.1);
        polyobst5->pushBackVertex(115.37, 80.1);
        polyobst5->finalizePolygon();
        obst_vector_.emplace_back(polyobst5);

        auto polyobst6 = boost::make_shared<PolygonObstacle>();
        polyobst6->pushBackVertex(118.77, 80.1);
        polyobst6->pushBackVertex(120, 80.1);
        polyobst6->pushBackVertex(120, 74.4);
        polyobst6->pushBackVertex(118.77, 74.4);

        polyobst6->finalizePolygon();
        obst_vector_.emplace_back(polyobst6);

        auto polyobst7 = boost::make_shared<PolygonObstacle>();
        polyobst7->pushBackVertex(108, 78.4);
        polyobst7->pushBackVertex(114.6, 78.4);
        polyobst7->pushBackVertex(114.6, 74.4);
        polyobst7->pushBackVertex(108, 74.4);

        polyobst7->finalizePolygon();
        obst_vector_.emplace_back(polyobst7);

        auto polyobst8 = boost::make_shared<PolygonObstacle>();
        polyobst8->pushBackVertex(114.6, 78.1);
        polyobst8->pushBackVertex(115.6, 78.1);
        polyobst8->pushBackVertex(115.6, 78.4);
        polyobst8->pushBackVertex(114.6, 78.4);

        polyobst8->finalizePolygon();
        obst_vector_.emplace_back(polyobst8);
    }

    bool NaviMasterRos::GlobalPlan()
    {
        Grid2D start, end;
        if (!costmap_2d_->WorldToMap(cur_pose_.pose.position.x, cur_pose_.pose.position.y, start.x, start.y))
        {
            ROS_INFO("THE cur pose is invalid");
            return false;
        }
        if (costmap_2d_->GridCost(start.x, start.y) != GridType::FREE)
        {
            ROS_INFO("THE cur pose is occupied, maybe location is wrong");
            return false;
        }
        if (!costmap_2d_->WorldToMap(target_pose_.pose.position.x, target_pose_.pose.position.y, end.x, end.y))
        {
            ROS_INFO("THE target pose is invalid");
            return false;
        }
        if (costmap_2d_->GridCost(end.x, end.y) != GridType::FREE)
        {
            ROS_INFO("THE target pose is occupied");
            return false;
        }
        global_planner_->SetCostmap2d(costmap_2d_);
        if (!global_planner_->Plan(start, end))
        {
            ROS_INFO("global plan failed");
            return false;
        }
        ROS_INFO("global plan success");
        ToNavPathMsg();
        return true;
    }

    void NaviMasterRos::ToNavPathMsg()
    {

        std::vector<Pose2D> global_path;
        global_planner_->GeneratePath(global_path);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.z = 0;
        global_path_.poses.clear();
        global_path_.poses.push_back(cur_pose_);

        for (int i = 1; i < global_path.size() - 1; i++)
        {

            pose_stamped.pose.position.x = global_path[i].x;
            pose_stamped.pose.position.y = global_path[i].y;
            global_path_.poses.push_back(pose_stamped);
        }
        global_path_.poses.push_back(target_pose_);
    }

    bool NaviMasterRos::LocalInitPlan()
    {
        std::vector<geometry_msgs::PoseStamped> initial_plan;
        for (int i = 0; i < global_path_.poses.size(); i++)
        {
            initial_plan.push_back(global_path_.poses.at(i));
        }
        ROS_INFO("local plan start");
        bool ans;
        ans = local_planner_->plan(initial_plan);
        if (!ans)
            return ans;
        ROS_INFO("local plan success");
        return true;
    }

    bool NaviMasterRos::LocalPlan()
    {
        PoseSE2 s(cur_pose_.pose);
        PoseSE2 e(target_pose_.pose);
        bool ans = local_planner_->plan(s, e);
        if (!ans)
            return ans;
        ROS_INFO("local real time plan success");
        return true;
    }

    void NaviMasterRos::GetControlVelocity()
    {
        LocalPlan();
        double vx, vy, omega;
        int look_ahead = 5;
        geometry_msgs::Twist vel;
        bool success = local_planner_->getVelocityCommand(vx, vy, omega, look_ahead);
        if (success)
        {
            Visualize(VisualizeType::PLAN);
            ROS_INFO("success calculate velocity");
            vel.linear.x = vx;
            vel.angular.z = omega;
            cmd_vel_pub_.publish(vel);
            bunker_cmd_vel_pub_.publish(vel);
            ROS_INFO("vel.linear:%0.2f, vel.angular:%0.2f", vel.linear.x, vel.angular.z);
            return;
        }
        ROS_INFO("fail calculate velocity");
    }

    void NaviMasterRos::Stop()
    {
        geometry_msgs::Twist vel;
        vel.linear.x = 0;
        vel.angular.z = 0;
        cmd_vel_pub_.publish(vel);
    }

    void NaviMasterRos::CurPoseCallBack(const nav_msgs::Odometry &odom_msg)
    {
        // ROS_INFO("get cur pose");
        // tf::trans()
        geometry_msgs::PoseStamped odom_pose;
        odom_pose.header = odom_msg.header;
        odom_pose.pose = odom_msg.pose.pose;
        //ROS_INFO("get cur pose x:%f",cur_pose_.pose.position.x);
        tf_.transform(odom_pose, cur_pose_, "map");
        //ROS_INFO("get cur pose x:%f",cur_pose_.pose.position.x);
        tf::Quaternion quat;
        tf::quaternionMsgToTF(cur_pose_.pose.orientation, quat);
        double roll, pitch, yaw;                      // 定义存储r\p\y的容器
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 进行转换
        double cur_theta = yaw;
        geometry_msgs::Pose2D pose2d;
        pose2d.x = cur_pose_.pose.position.x;
        pose2d.y = cur_pose_.pose.position.y;
        pose2d.theta = cur_theta;
        cur_pose_pub_.publish(pose2d);
        // cur_pose_.pose = odom_msg.pose.pose;
        cur_vel_ = odom_msg.twist.twist;
    }

    void NaviMasterRos::TargetPoseCallBack(const geometry_msgs::PoseStamped &pose_msg)
    {
        // if(pose_msg.pose.position.x == 78 && pose_msg.pose.position.y == 4)
        // {
        //     navi_cmd_ = NaviCommand::PIDseed;
        //     return;
        // }
        target_pose_ = pose_msg;
        navi_cmd_ = NaviCommand::NaviPlanCmd;
    }

    void NaviMasterRos::RvizClickPointCallBack(const geometry_msgs::PointStamped &pointstamped)
    {
        ROS_INFO("we have receive the click point from rviz");
        cur_pose_.pose.position = pointstamped.point;
        cur_pose_.pose.orientation.w = 0;
        cur_pose_.pose.orientation.x = 0;
        cur_pose_.pose.orientation.y = 0;
        cur_pose_.pose.orientation.z = 0;
    }

    void NaviMasterRos::NaviCmdCallBack(const std_msgs::Int8 &cmd)
    {
        if (cmd.data > 0 && cmd.data < 11)
        {
            int data = cmd.data;
            navi_cmd_ = (NaviCommand)data;
        }
    }

    bool NaviMasterRos::IsArriveTarget(geometry_msgs::Pose &target_pose, double xy_tolerance, double theta_tolerance)
    {
        double dx = target_pose.position.x - cur_pose_.pose.position.x;
        double dy = target_pose.position.y - cur_pose_.pose.position.y;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(cur_pose_.pose.orientation, quat);
        double roll, pitch, yaw;                      // 定义存储r\p\y的容器
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 进行转换
        double cur_theta = yaw;
        tf::quaternionMsgToTF(target_pose.orientation, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 进行转换
        double target_theta = yaw;
        if (hypot(dx, dy) < xy_tolerance && abs(cur_theta - target_theta) < theta_tolerance)
        {
            return true;
        }
        return false;
    }

    bool NaviMasterRos::IsNaviArriveTarget()
    {
        return IsArriveTarget(target_pose_.pose, config_.goal_tolerance.xy_goal_tolerance, config_.goal_tolerance.yaw_goal_tolerance);
    }

    void NaviMasterRos::Visualize(VisualizeType visulize_type)
    {
        switch (visulize_type)
        {
        case VisualizeType::COSTMAP:
            costmap_2d_ros_->Visualize();
            break;
        case VisualizeType::PLAN:
            static int seq = 0;
            global_path_.header.stamp = ros::Time::now();
            global_path_.header.seq = ++seq;
            global_path_pub_.publish(global_path_);
            // local_path_pub_.publish(local_path_);
            local_planner_->visualize();
            visual_->publishObstacles(obst_vector_);
            break;
        default:
            break;
        }
    }

    void NaviMasterRos::SetNaviCmd(NaviCommand cmd)
    {
        navi_cmd_ = cmd;

        std_msgs::Int8 state_cmd_pub;
        switch (cmd)
        {
        case WaitCmd:
            state_cmd_pub.data = 1;
            state_pub_.publish(state_cmd_pub);
            break;
        case NaviPlanCmd:
            state_cmd_pub.data = 2;
            state_pub_.publish(state_cmd_pub);
            break;
        case NaviControlCmd:
            state_cmd_pub.data = 3;
            state_pub_.publish(state_cmd_pub);
            break;
        case ReInitCmd:
            state_cmd_pub.data = 4;
            state_pub_.publish(state_cmd_pub);
            break;
        case StopCmd:
            state_cmd_pub.data = 1;
            state_pub_.publish(state_cmd_pub);
            break;
        case LineControlCmd:
            state_cmd_pub.data = 6;
            state_pub_.publish(state_cmd_pub);
            break;
        case RotateControlCmd:
            state_cmd_pub.data = 7;
            state_pub_.publish(state_cmd_pub);
            break;
        case StratCmd:
            state_cmd_pub.data = 8;
            state_pub_.publish(state_cmd_pub);
            break;
        case TurnCmd:
            state_cmd_pub.data = 9;
            state_pub_.publish(state_cmd_pub);
            break;
        case PIDseed:
            state_cmd_pub.data = 6;
            state_pub_.publish(state_cmd_pub);
            break;
        default:
            break;
        }
    }

    void NaviMasterRos::SeedLineVisualize()
    {
        int idx = 0;
        geometry_msgs::PoseArray line_dir;
        line_dir.header.frame_id = "map";
        geometry_msgs::Pose2D line_start, line_end;
        for (int i = 1; i <= 9; i++)
        {
            for (int j = 1; j <= 10; j++)
            {

                SeedLinePose(i, j, line_start, line_end);
                geometry_msgs::Quaternion geo_q = tf::createQuaternionMsgFromYaw(line_start.theta);
                geometry_msgs::Pose cur;
                cur.position.x = line_start.x;
                cur.position.y = line_start.y;
                cur.position.z = 0;
                cur.orientation = geo_q;
                line_dir.poses.push_back(cur);
                cur.position.x = line_end.x;
                cur.position.y = line_end.y;
                line_dir.poses.push_back(cur);
                cur.position.x = (line_end.x + line_start.x) / 2;
                cur.position.y = (line_end.y + line_start.y) / 2;
                line_dir.poses.push_back(cur);
                visualization_msgs::Marker marker;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time::now();
                marker.ns = "SeedLine";
                marker.id = idx++;
                marker.type = visualization_msgs::Marker::LINE_STRIP;
                marker.action = visualization_msgs::Marker::ADD;
                marker.lifetime = ros::Duration(2.0);
                geometry_msgs::Point start;
                start.x = line_start.x;
                start.y = line_start.y;
                start.z = 0;
                marker.points.push_back(start);
                geometry_msgs::Point end;
                end.x = line_end.x;
                end.y = line_end.y;
                end.z = 0;
                marker.points.push_back(end);
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                line_pose_pub_.publish(marker);
            }
        }
        line_dir_pub_.publish(line_dir);
    }

    // 直线行驶控制输出计算，需要根据方向设定直线的倾角heading
    void NaviMasterRos::LineControl(double heading)
    {
        pid_control_->LineControlOutput(cur_pose_.pose, cur_vel_, heading);
        cmd_vel_pub_.publish(cur_vel_);
        bunker_cmd_vel_pub_.publish(cur_vel_);
    }
    void NaviMasterRos::RotateControl(double target_theta)
    {
        pid_control_->RotateControlOutput(cur_pose_.pose, target_theta, cur_vel_);
        cmd_vel_pub_.publish(cur_vel_);
        bunker_cmd_vel_pub_.publish(cur_vel_);
    }
    void NaviMasterRos::PubPIDGoal(double x, double y, double theta)
    {
        geometry_msgs::Quaternion q;
        q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, theta);
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = ros::Time::now();
        goal_pose.pose.position.x = x;
        goal_pose.pose.position.y = y;
        goal_pose.pose.position.z = 0;
        // goal_pose.pose.orientation.z = 0.711668009588;
        // goal_pose.pose.orientation.w = 0.702515938701;
        goal_pose.pose.orientation.x = q.x;
        goal_pose.pose.orientation.y = q.y;
        goal_pose.pose.orientation.z = q.z;
        goal_pose.pose.orientation.w = q.w;
        // while(publish.getNumSubscribers()<1);
        publish_PID_goal.publish(goal_pose);
    }

    bool NaviMasterRos::SeedIsLineArriveTerminal(int dir, int idx)
    {
        if (dir == 1) // X方向 转弯后去往下一垄的起点
        {
            if (abs(cur_pose_.pose.position.x - seed_LineXYTerminal[idx][dir]) < linecontrol_xy_tolerance_)
            {
                return true;
            }
            return false;
        }
        else if (dir == 0) // Y方向 播种的终点
        {
            if (abs(cur_pose_.pose.position.y - seed_LineXYTerminal[idx][dir]) < linecontrol_xy_tolerance_)
            {
                return true;
            }
            return false;
        }
    }

    bool NaviMasterRos::SeedIsRotateArriveTerminal(int idx)
    {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(cur_pose_.pose.orientation, quat);
        double roll, pitch, yaw;                      // 定义存储r\p\y的容器
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 进行转换
        double cur_theta = yaw;
        if (abs(cur_theta - RotateTargetTheta[idx]) < rotate_theta_tolerance_)
        {
            return true;
        }
        return false;
    }

    bool NaviMasterRos::IsLineArriveTerminal(int dir, int idx)
    {
        if (dir == 1) // X方向 转弯后去往下一垄的起点
        {
            if (abs(cur_pose_.pose.position.x - seed_LineXYTerminal[idx][dir]) < linecontrol_xy_tolerance_)
            {
                return true;
            }
            return false;
        }
        else if (dir == 0) // Y方向 播种的终点
        {
            if (abs(cur_pose_.pose.position.y - seed_LineXYTerminal[idx][dir]) < linecontrol_xy_tolerance_)
            {
                return true;
            }
            return false;
        }
    }

    bool NaviMasterRos::IsRotateArriveTerminal(int idx)
    {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(cur_pose_.pose.orientation, quat);
        double roll, pitch, yaw;                      // 定义存储r\p\y的容器
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 进行转换
        double cur_theta = yaw;
        if (abs(cur_theta - seedlineTheta[idx]) < rotate_theta_tolerance_)
        {
            return true;
        }
        return false;
    }

} // namespace navigation
