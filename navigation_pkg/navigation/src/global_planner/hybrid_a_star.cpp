#include "hybrid_a_star.h"
#include "math.h"

namespace navigation
{

    NodeHAsPtr HybridAstar::CreateNodeFromMap(const Cord &cord)
    {
        return CreateNodeFromMap(cord.x, cord.y, cord.theta);
    }

    NodeHAsPtr HybridAstar::CreateNodeFromMap(const int &x, const int &y, const int &theta)
    {
        NodeHAsPtr node = std::make_shared<NodeHAs>(x, y, theta);
        node_set_[node->NodeHAsTag()] = node;

        return node;
    }

    NodeHAsPtr HybridAstar::InNodeSet(const std::string &tag)
    {
        if (node_set_.find(tag) == node_set_.end())
        {
            return nullptr;
        }
        return node_set_[tag];
    }

    NodeHAsPtr HybridAstar::GetNodeFromMap(const int &x, const int &y, const int &theta)
    {
        std::string tag = std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(theta);
        NodeHAsPtr node = InNodeSet(tag);
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

    NodeHAsPtr HybridAstar::GetNodeFromMap(const Cord &cord)
    {
        return GetNodeFromMap(cord.x, cord.y, cord.theta);
    }

    NodeHAsPtr HybridAstar::GetNodeFromWorld(const Pose2D &pose)
    {
        int x, y, theta;
        costmap_2d_->WorldToMap(pose.x, pose.y, x, y);
        double w_t = pose.theta;
        NormalizeTheta(w_t);
        theta = static_cast<int>(w_t / phi_resolution_);
        return GetNodeFromMap(x, y, theta);
    }

    std::string HybridAstar::CalculateNode3DTag(const Pose2D &pos)
    {
        int x, y, theta;
        costmap_2d_->WorldToMap(pos.x, pos.y, x, y);
        double w_t = pos.theta;
        NormalizeTheta(w_t);
        theta = static_cast<int>(w_t / phi_resolution_);
        std::string tag = std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(theta);
        return tag;
    }

    void HybridAstar::NormalizeTheta(double &theta)
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

    void HybridAstar::Init()
    {
        ResetSearch();
    }

    void HybridAstar::ResetSearch()
    {
        closed_set_.clear();
        node_set_.clear();
        decltype(open_pq_) null_pq;
        open_pq_.swap(null_pq);
    }

    bool HybridAstar::Plan()
    {
        ResetSearch();
        // printf("11\n");
        NodeHAsPtr start_node = GetNodeFromWorld(start_pose_);
        start_node->SetGCost(0);
        start_node->SetNodePose(start_pose_);
        start_node->SetNodeDir(Direction::Forward);
        open_pq_.push({start_node->NodeHAsTag(), CalculateKey(start_node)});
        NodeHAsPtr end_node = GetNodeFromWorld(end_pose_);
        while (!open_pq_.empty())
        {
            TagKey cur_tagkey = open_pq_.top();
            open_pq_.pop();
            std::string cur_node_tag = cur_tagkey.first;
            // 在闭集里面，不需要再考虑。
            if (closed_set_.find(cur_node_tag) != closed_set_.end())
            {
                continue;
            }
            //得到当前node
            NodeHAsPtr cur_node = node_set_[cur_node_tag];
            if (cur_node->Occupy())
            {
                std::cout << "not consider occupy" << std::endl;
                return false;
            }
            // std::cout<<"x: "<<cur_node->NodePose().x<<"y: "<<cur_node->NodePose().y<<"theta: "<<cur_node->NodePose().theta<<std::endl;
            //加入闭集———加入进去的不一定是最优解
            closed_set_[cur_node_tag] = cur_node;
            //是终点了，可以退出
            if (IsArriveTarget(cur_node))
            {
                if (end_node->NodeHAsTag() != cur_node->NodeHAsTag())
                {
                    end_node->SetParent(cur_node_tag);
                }
                return true;
            }
            //搜索
            std::vector<Pose2D> next_poses;
            GenNextNodes(cur_node, next_poses, Direction::Forward);
            GenNextNodes(cur_node, next_poses, Direction::Circle);
            GenNextNodes(cur_node, next_poses, Direction::Backward);
            for (int i = 0; i < next_poses.size(); i++)
            {
                auto next_pose = next_poses[i];
                //确保node_set里面的是最新的
                std::string next_tag = CalculateNode3DTag(next_pose);
                NodeHAsPtr next_node = GetNodeFromWorld(next_pose);
                Direction dir = Direction::Forward;
                double dis_cost = step_dis_;
                if (i >= next_node_num_)
                {
                    dir = Direction::Circle;
                    dis_cost = std::abs(max_angle_);
                    if (i >= 2 * next_node_num_)
                    {
                        dir = Direction::Backward;
                        dis_cost = step_dis_;
                    }
                }
                dis_cost += StepCost(cur_node, next_node);
                double dir_cost = (dir == cur_node->NodeDir() ? 0 : penalty);
                double next_g = cur_node->GCost() + dir_cost + dis_cost;
                //如果这个点和现在的点索引一样 那么只有当这个点的启发代价比现在点的启发代价小的时候才会更新
                if (next_tag == cur_node->NodeHAsTag())
                {
                    // double cost = next_g + HeuristicCost(next_pose);
                    // if (cost < CalculateKey(cur_node))
                    // {
                    //     next_node->SetGCost(next_g);
                    //     next_node->SetNodePose(next_pose);
                    //     next_node->SetNodeDir(dir);
                    //     next_node->SetParent(cur_node->Parent());
                    //     open_pq_.push({next_node->NodeHAsTag(), CalculateKey(start_node)});
                    //     closed_set_.erase(next_node->NodeHAsTag());
                    // }
                }
                else //索引不一样 比较G
                {
                    if (next_g < next_node->GCost())
                    {
                        next_node->SetGCost(next_g);
                        next_node->SetNodePose(next_pose);
                        next_node->SetNodeDir(dir);
                        next_node->SetParent(cur_node->NodeHAsTag());
                        open_pq_.push({next_node->NodeHAsTag(), CalculateKey(next_node)});
                        if (closed_set_.find(next_node->NodeHAsTag()) != closed_set_.end())
                        {
                            closed_set_.erase(next_node->NodeHAsTag());
                        }
                    }
                }
            }
        }
        return false;
    }

    bool HybridAstar::Plan(Cord &start, Cord &end)
    {
        SetStartGird(start);
        SetEndGird(end);
        costmap_2d_->MapToWorld(start.x, start.y, start_pose_.x, start_pose_.y);
        costmap_2d_->MapToWorld(end.x, end.y, end_pose_.x, end_pose_.y);
        start_pose_.theta = start.theta * phi_resolution_;
        end_pose_.theta = end.theta * phi_resolution_;
        return Plan();
    }

    bool HybridAstar::Plan(Pose2D &start, Pose2D &end)
    {
        start_pose_ = start;
        end_pose_ = end;
        return Plan();
    }

    double HybridAstar::StepCost(NodeHAsPtr &start_node, NodeHAsPtr &end_node)
    {
        if (start_node->Occupy() || end_node->Occupy())
        {
            return std::numeric_limits<double>::infinity();
        }
        // Cord step = start_node->Cordinate() - end_node->Cordinate();
        return 0;
    }

    bool HybridAstar::IsArriveTarget(NodeHAsPtr &node3d)
    {
        double delta_x = node3d->NodePose().x - end_pose_.x;
        double delta_y = node3d->NodePose().y - end_pose_.y;
        double delta_theta = node3d->NodePose().theta - end_pose_.theta;
        if (std::hypot(delta_x, delta_y) <= goal_xy_tolerant_ && std::abs(delta_theta) <= goal_theta_tolerant_)
        {
            return true;
        }
        return false;
    }

    void HybridAstar::GenNextNodes(NodeHAsPtr &cur_node, std::vector<Pose2D> &next_poses, Direction direction)
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

    void HybridAstar::VehicleKinematics(const Pose2D &pose, Pose2D &new_pose, double dis, double theta)
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
        double d_x = r_x * std::cos(pose.theta) - r_y * std::sin(pose.theta) ;
        double d_y = r_x * std::sin(pose.theta) + r_y * std::cos(pose.theta) ;
        new_pose.x = pose.x + d_x;
        new_pose.y = pose.y + d_y;
        new_pose.theta = pose.theta + theta;
    }

    void HybridAstar::GeneratePath(std::vector<Pose2D> &path)
    {
        NodeHAsPtr cur = GetNodeFromWorld(end_pose_);
        NodeHAsPtr start = GetNodeFromWorld(start_pose_);
        path.push_back(cur->NodePose());
        while (cur != start)
        {
            std::string tag = cur->Parent();
            // std::cout << tag << std::endl;
            cur = node_set_[tag];
            path.push_back(cur->NodePose());
        }
        std::reverse(path.begin(), path.end());
    }

} // namespace navigation
