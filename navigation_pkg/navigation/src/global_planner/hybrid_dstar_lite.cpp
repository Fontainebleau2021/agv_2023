
#include "hybrid_dstar_lite.h"

namespace navigation
{
     NodeHDsPtr HybridDStarLite::CreateNodeFromMap(const Cord &cord)
    {
        return CreateNodeFromMap(cord.x, cord.y, cord.theta);
    }

    NodeHDsPtr HybridDStarLite::CreateNodeFromMap(const int &x, const int &y, const int &theta)
    {
        NodeHDsPtr node = std::make_shared<NodeHDs>(x, y, theta);
        node_set_[node->NodeHDsTag()] = node;
        return node;
    }

    NodeHDsPtr HybridDStarLite::InNodeSet(const std::string &tag)
    {
        if (node_set_.find(tag) == node_set_.end())
        {
            return nullptr;
        }
        return node_set_[tag];
    }

    NodeHDsPtr HybridDStarLite::GetNodeFromMap(const int &x, const int &y, const int &theta)
    {
        std::string tag = std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(theta);
        NodeHDsPtr node = InNodeSet(tag);
        if (node == nullptr)
        {
            node = CreateNodeFromMap(x, y, theta);
        }
        if (costmap_2d_->GridCost(x, y) == GridType::OBSTACLE)
        {
            node->SetOccupy(true);
        }
        else
        {
            node->SetOccupy(false);
        }
        return node;
    }

    NodeHDsPtr HybridDStarLite::GetNodeFromMap(const Cord &cord)
    {
        return GetNodeFromMap(cord.x, cord.y, cord.theta);
    }

    NodeHDsPtr HybridDStarLite::GetNodeFromWorld(const Pose2D &pose)
    {
        int x, y, theta;
        costmap_2d_->WorldToMap(pose.x, pose.y, x, y);
        double w_t = pose.theta;
        NormalizeTheta(w_t);
        theta = static_cast<int>(w_t / phi_resolution_);
        return GetNodeFromMap(x, y, theta);
    }

    void HybridDStarLite::NormalizeTheta(double &theta)
    {
        while (theta > M_PI)
        {
            theta -= 2 * M_PI;
        }
        while (theta <= -M_PI)
        {
            theta += 2 * M_PI;
        }
    }

    void HybridDStarLite::UpdateVertex(NodeHDsPtr node)
    {

        if (node->Cordinate() != end_node_->Cordinate())
        {
            double min_gcost = std::numeric_limits<double>::infinity();
            std::string parent;
            std::vector<Pose2D> next_poses;
            GenNextPoses(node, next_poses, Direction::Forward);
            GenNextPoses(node, next_poses, Direction::Circle);
            GenNextPoses(node, next_poses, Direction::Backward);
            for (int i = 0; i < next_poses.size(); i++)
            {
                auto next_pose = next_poses[i];
                //确保node_set里面的是最新的
                // std::string next_tag = CalculateNode3DTag(next_pose);
                NodeHDsPtr prev_node = GetNodeFromWorld(next_pose);
                // UpdateVertex(next_node);
                Direction dir = Direction::Backward;
                double dis_cost = step_dis_;
                if (i >= next_node_num_)
                {
                    dir = Direction::Circle;
                    dis_cost = std::abs(max_angle_);
                    if (i >= 2 * next_node_num_)
                    {
                        dir = Direction::Forward;
                        dis_cost = step_dis_;
                    }
                }
                dis_cost += StepCost(prev_node, node);
                double dir_cost = (dir == node->NodeDir() ? 0 : penalty);
                double next_g = prev_node->GCost() + dir_cost + dis_cost;
                if (next_g < min_gcost)
                {
                    min_gcost = next_g;
                    parent = prev_node->NodeHDsTag();
                }
                min_gcost = std::min(min_gcost, next_g);
            }
            node->SetrhsCost(min_gcost);
            node->SetParent(parent);
        }

        if (open_set_.find(node->NodeHDsTag()) != open_set_.end())
        {
            open_set_.erase(node->NodeHDsTag());
        }

        if (node->rhsCost() != node->GCost())
        {
            open_pq_.push({node->NodeHDsTag(), CalculateKey(node)});
            open_set_.insert(node->NodeHDsTag());
        }
    }

    double HybridDStarLite::StepCost(NodeHDsPtr &start_node, NodeHDsPtr &end_node)
    {
        if (start_node->Occupy() || end_node->Occupy())
        {
            return std::numeric_limits<double>::infinity();
        }
        //Cord step = start_node->Cordinate() - end_node->Cordinate();
        return 0;
    }

    void HybridDStarLite::Init()
    {
        end_node_ = GetNodeFromWorld(end_pose_);
        end_node_->SetrhsCost(0);
        open_pq_.push({end_node_->NodeHDsTag(), CalculateKey(end_node_)});
        open_set_.insert(end_node_->NodeHDsTag());
        initialized_ = true;
        // printf("111")
    }

    bool HybridDStarLite::Plan()
    {

        //UpdateEdgeCost();
        NodeHDsPtr start_node = GetNodeFromWorld(start_pose_);
        while (!open_pq_.empty() && ((open_pq_.top().second < CalculateKey(start_node)) || (start_node->rhsCost() != start_node->GCost())))
        {
            HDsTagKey cur_tagkey = open_pq_.top();
            open_pq_.pop();
            std::string cur_node_tag = cur_tagkey.first;
            //不在openset里面
            if (open_set_.find(cur_node_tag) == open_set_.end())
            {
                continue;
            }
            open_set_.erase(cur_node_tag);
            NodeHDsPtr cur_node = node_set_[cur_node_tag];
            HDsKey k_old = cur_tagkey.second;
            if (k_old < CalculateKey(cur_node))
            {
                open_pq_.push({cur_node->NodeHDsTag(), CalculateKey(cur_node)});
                open_set_.insert(cur_node->NodeHDsTag());
            }
            else if (cur_node->GCost() > cur_node->rhsCost())
            {
                cur_node->SetGCost(cur_node->rhsCost());
                std::vector<Pose2D> next_poses;
                GenNextPoses(cur_node, next_poses, Direction::Forward);
                GenNextPoses(cur_node, next_poses, Direction::Circle);
                GenNextPoses(cur_node, next_poses, Direction::Backward);
                for (int i = 0; i < next_poses.size(); i++)
                {
                    Direction dir = Direction::Forward;
                    // double dis_cost = step_dis_;
                    if (i >= next_node_num_)
                    {
                        dir = Direction::Circle;
                        // dis_cost = std::abs(max_angle_);
                        if (i >= 2 * next_node_num_)
                        {
                            dir = Direction::Backward;
                            // dis_cost = step_dis_;
                        }
                    }
                    auto next_pose = next_poses[i];
                    //确保node_set里面的是最新的
                    // std::string next_tag = CalculateNode3DTag(next_pose);
                    NodeHDsPtr next_node = GetNodeFromWorld(next_pose);
                    if (next_node->NodeHDsTag() == cur_node->NodeHDsTag())
                    {
                        continue;
                    }
                    next_node->SetNodeDir(dir);
                    UpdateVertex(next_node);
                }
                // for (int i = 0; i < 8; i++)
                // {
                //     Cord next_cord = cur_node->Cordinate() + dir_[i];
                //     if (!costmap_2d_->Valid(next_cord.x, next_cord.y))
                //     {
                //         continue;
                //     }
                //     NodeHDsPtr next_node = GetNodeFromMap(next_cord);
                //     UpdateVertex(next_node);
                // }
            }
            else
            {
                cur_node->SetGCost(std::numeric_limits<double>::infinity());
                UpdateVertex(cur_node);
                std::vector<Pose2D> next_poses;
                GenNextPoses(cur_node, next_poses, Direction::Forward);
                GenNextPoses(cur_node, next_poses, Direction::Circle);
                GenNextPoses(cur_node, next_poses, Direction::Backward);
                for (int i = 0; i < next_poses.size(); i++)
                {
                    Direction dir = Direction::Forward;
                    // double dis_cost = step_dis_;
                    if (i >= next_node_num_)
                    {
                        dir = Direction::Circle;
                        // dis_cost = std::abs(max_angle_);
                        if (i >= 2 * next_node_num_)
                        {
                            dir = Direction::Backward;
                            // dis_cost = step_dis_;
                        }
                    }
                    auto next_pose = next_poses[i];
                    //确保node_set里面的是最新的
                    // std::string next_tag = CalculateNode3DTag(next_pose);
                    NodeHDsPtr next_node = GetNodeFromWorld(next_pose);
                    if (next_node->NodeHDsTag() == cur_node->NodeHDsTag())
                    {
                        continue;
                    }
                    next_node->SetNodeDir(dir);
                    UpdateVertex(next_node);
                }
            }
        }
        if (start_node->rhsCost() == start_node->GCost())
        {
            return true;
        }
        return false;
    }

    bool HybridDStarLite::Plan(Cord &start, Cord &end)
    {
        if (!initialized_)
        {
            SetStartGird(start);
            SetEndGird(end);
            costmap_2d_->MapToWorld(start.x, start.y, start_pose_.x, start_pose_.y);
            costmap_2d_->MapToWorld(end.x, end.y, end_pose_.x, end_pose_.y);
            start_pose_.theta = start.theta * phi_resolution_;
            end_pose_.theta = end.theta * phi_resolution_;
            Init();
        }

        return Plan();
    }

    bool HybridDStarLite::Plan(Pose2D &start, Pose2D &end)
    {
        if (!initialized_)
        {
            start_pose_ = start;
            end_pose_ = end;
            Init();
        }
        return Plan();
    }

    void HybridDStarLite::GeneratePath(std::vector<Pose2D> &path)
    {
        // NodeHDsPtr cur = GetNodeFromWorld(end_pose_);
        // NodeHDsPtr start = GetNodeFromWorld(start_pose_);
        // path.push_back(cur->NodePose());
        // while (cur != start)
        // {
        //     std::string tag = cur->Parent();
        //     // std::cout << tag << std::endl;
        //     cur = node_set_[tag];
        //     path.push_back(cur->NodePose());
        // }
        // std::reverse(path.begin(), path.end());
    }

    void HybridDStarLite::GenNextPoses(NodeHDsPtr &cur_node, std::vector<Pose2D> &next_poses, Direction direction)
    {
        double dis = 0.0;
        switch (direction)
        {
        case Direction::Forward:
            dis = step_dis_;
            break;
        case Direction::Backward:
            dis = -step_dis_;
            break;
        case Direction::Circle:
            dis = 0.0;
        default:
            break;
        }
        Pose2D new_pose;
        int x, y;
        for (int num = 0; num < next_node_num_; num++)
        {
            double angle = -max_angle_ + 2 * max_angle_ * num / (next_node_num_ - 1);
            VehicleKinematics(cur_node->NodePose(), new_pose, dis, angle);
            if (!costmap_2d_->WorldToMap(new_pose.x, new_pose.y, x, y))
            {
                continue;
            }
            NormalizeTheta(new_pose.theta);
            next_poses.push_back(new_pose);
        }
    }

    void HybridDStarLite::VehicleKinematics(const Pose2D &pose, Pose2D &new_pose, double dis, double theta)
    {
        double r, r_x, r_y;
        if (theta != 0)
        {
            r = dis / theta;
            r_x = r * std::sin(theta), r_y = r * (1 - std::cos(theta));
        }
        else
        {
            r_x = dis;
            r_y = 0;
        }
        // double r = dis / theta;
        // double r_x = r * std::sin(theta), r_y = dis * (1 - std::cos(theta));
        // //机器人坐标系中下一个点的位置
        // double r_x = dis * std::cos(theta), r_y = dis * std::sin(theta);
        // //机器人坐标系与世界坐标系进行旋转变换
        double d_x = r_x * std::cos(pose.theta) - r_y * std::sin(pose.theta) + pose.x;
        double d_y = r_x * std::sin(pose.theta) + r_y * std::cos(pose.theta) + pose.y;
        new_pose.x = pose.x + d_x;
        new_pose.y = pose.y + d_y;
        new_pose.theta = pose.theta + theta;
    }

    void HybridDStarLite::UpdateEdgeCost()
    {
        
    }

} // namespace planner
