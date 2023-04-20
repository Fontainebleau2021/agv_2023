#include "LPAstar.h"

namespace navigation
{

    NodeDsPtr LPAstar::CreateNodeFromMap(const Cord &pos)
    {
        return CreateNodeFromMap(pos.x, pos.y);
    }

    NodeDsPtr LPAstar::CreateNodeFromMap(const int &x, const int &y)
    {
        NodeDsPtr node = std::make_shared<NodeDs>(x, y);
        node_set_[node->Node2DTag()] = node;
        return node;
    }

    NodeDsPtr LPAstar::InNodeSet(std::string &tag)
    {
        if (node_set_.find(tag) == node_set_.end())
        {
            return nullptr;
        }
        return node_set_[tag];
    }

    NodeDsPtr LPAstar::GetNodeFromMap(const int &x, const int &y)
    {
        std::string tag = std::to_string(x) + "_" + std::to_string(y);
        NodeDsPtr node = InNodeSet(tag);
        if (node == nullptr)
        {
            node = CreateNodeFromMap(x, y);
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

    NodeDsPtr LPAstar::GetNodeFromMap(const Cord &pos)
    {
        return GetNodeFromMap(pos.x, pos.y);
    }

    void LPAstar::UpdateVertex(NodeDsPtr &node)
    {

        if (node->Cordinate() != start_grid_)
        {
            double min_gcost = std::numeric_limits<double>::infinity();
            std::string parent;
            for (int i = 0; i < 8; i++)
            {
                Cord prev_cord = node->Cordinate() + dir_[i];
                if (!costmap_2d_->Valid(prev_cord.x, prev_cord.y))
                {
                    continue;
                }
                NodeDsPtr prev_node = GetNodeFromMap(prev_cord);
                double step_dist = StepCost(prev_node, node);
                // double next_gcost = node->GCost() + step_dist;
                // if (prev_node->GCost() + step_dist < min_gcost)
                // {
                //     min_gcost = prev_node->GCost() + step_dist;
                // }
                min_gcost = std::min(min_gcost, prev_node->GCost() + step_dist);
            }
            node->SetrhsCost(min_gcost);
            // node->SetParent(parent);
        }
        if (open_set_.find(node->Node2DTag()) != open_set_.end())
        {
            open_set_.erase(node->Node2DTag());
        }
        if (node->rhsCost() != node->GCost())
        {
            //printf("111\n");
            open_pq_.push({node->Node2DTag(), CalculateKey(node)});
            open_set_.insert(node->Node2DTag());
        }
    }

    double LPAstar::StepCost(NodeDsPtr &start_node, NodeDsPtr &end_node)
    {
        if (start_node->Occupy() || end_node->Occupy())
        {
            return std::numeric_limits<double>::infinity();
        }
        Cord step = start_node->Cordinate() - end_node->Cordinate();
        return step.norm();
    }

    void LPAstar::Init()
    {
        NodeDsPtr start_node = GetNodeFromMap(start_grid_);
        start_node->SetrhsCost(0);
        open_pq_.push({start_node->Node2DTag(), CalculateKey(start_node)});
        open_set_.insert(start_node->Node2DTag());
        initialized_ = true;
        // printf("111")
    }

    bool LPAstar::Plan()
    {

        //NodeDsPtr start_node = GetNodeFromMap(start_grid_);
        UpdateEdgeCost();
        // initialized_ = true;
        NodeDsPtr end_node = GetNodeFromMap(end_grid_);
        while (!open_pq_.empty() && ((open_pq_.top().second < CalculateKey(end_node)) || (end_node->rhsCost() != end_node->GCost())))
        {
            //printf("into while\n");
            LPAsTagKey cur_tagkey = open_pq_.top();
            open_pq_.pop();
            std::string cur_node_tag = cur_tagkey.first;
            if (open_set_.find(cur_node_tag) == open_set_.end())
            {
                continue;
            }
            open_set_.erase(cur_node_tag);
            NodeDsPtr cur_node = node_set_[cur_node_tag];
            if (cur_node->GCost() > cur_node->rhsCost())
            {
                cur_node->SetGCost(cur_node->rhsCost());
            }
            else
            {
                cur_node->SetGCost(std::numeric_limits<double>::infinity());
                UpdateVertex(cur_node);
            }

            for (int i = 0; i < 8; i++)
            {
                Cord next_cord = cur_node->Cordinate() + dir_[i];
                if (!costmap_2d_->Valid(next_cord.x, next_cord.y))
                {
                    continue;
                }
                NodeDsPtr next_node = GetNodeFromMap(next_cord);
                UpdateVertex(next_node);
            }
            //加入闭集———加入进去的不一定是最优解
            // closed_set_[cur_node_tag] = cur_node;
        }
        if (end_node->rhsCost() == end_node->GCost())
        {
            // GeneratePath();
            return true;
        }
        return false;
    }

    bool LPAstar::Plan(Cord &start, Cord &end)
    {
        if (!initialized_)
        {
            SetStartGird(start);
            SetEndGird(end);
            Init();
        }

        return Plan();
    }

    bool LPAstar::Plan(Pose2D &start, Pose2D &end)
    {
        if (!initialized_)
        {
            costmap_2d_->WorldToMap(start.x, start.y, start_grid_.x, start_grid_.y);
            costmap_2d_->WorldToMap(end.x, end.y, end_grid_.x, end_grid_.y);
            Init();
        }
        return Plan();
    }

    void LPAstar::GeneratePath(std::vector<Pose2D> &path)
    {
        NodeDsPtr cur = GetNodeFromMap(end_grid_);
        NodeDsPtr start = GetNodeFromMap(start_grid_);
        Pose2D pose;
        std::string tag;
        //NodeDsPtr min_g_node = std::make_shared<NodeDs>(-1, -1);
        while (cur != start)
        {
            costmap_2d_->MapToWorld(cur->Cordinate().x, cur->Cordinate().y, pose.x, pose.y);
            path.push_back(pose);
            //min_g_node = std::make_shared<NodeDs>(-1, -1);
            double cost = std::numeric_limits<double>::infinity();
            for (int i = 0; i < 8; i++)
            {
                Cord next_cord = cur->Cordinate() + dir_[i];
                if (!costmap_2d_->Valid(next_cord.x, next_cord.y))
                {
                    continue;
                }
                NodeDsPtr next_node = GetNodeFromMap(next_cord);
                if (cost > next_node->GCost())
                {
                    cost = next_node->GCost();
                    tag = next_node->Node2DTag();
                }
            }
            cur = node_set_[tag];
        }
        costmap_2d_->MapToWorld(cur->Cordinate().x, cur->Cordinate().y, pose.x, pose.y);
        path.push_back(pose);
        std::reverse(path.begin(), path.end());
    }

    void LPAstar::UpdateEdgeCost()
    {
        auto delete_cords = costmap_2d_->DeleteObs();
        for (int i = 0; i < delete_cords.size(); i++)
        {
            //printf("into delete obs update\n");
            NodeDsPtr node = GetNodeFromMap(delete_cords[i]);
            UpdateVertex(node);
            for (int i = 0; i < 8; i++)
            {
                Cord next_cord = node->Cordinate() + dir_[i];
                if (!costmap_2d_->Valid(next_cord.x, next_cord.y))
                {
                    continue;
                }
                NodeDsPtr next_node = GetNodeFromMap(next_cord);
                UpdateVertex(next_node);
            }
        }
        auto obs_cords = costmap_2d_->NewObs();
        for (int i = 0; i < obs_cords.size(); i++)
        {
           
            NodeDsPtr node = GetNodeFromMap(obs_cords[i]);
            for (int i = 0; i < 8; i++)
            {
                Cord next_cord = node->Cordinate() + dir_[i];
                if (!costmap_2d_->Valid(next_cord.x, next_cord.y))
                {
                    continue;
                }
                NodeDsPtr next_node = GetNodeFromMap(next_cord); 
                //printf("into obs update\n");
                UpdateVertex(next_node);
            }
        }
    }
} // namespace planner
