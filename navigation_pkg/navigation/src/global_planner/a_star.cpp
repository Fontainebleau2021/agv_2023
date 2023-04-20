
#include "a_star.h"
#include <algorithm>

namespace navigation
{

    NodeAsPtr Astar::CreateNodeFromMap(const Cord &pos)
    {
        return CreateNodeFromMap(pos.x, pos.y);
    }

    NodeAsPtr Astar::CreateNodeFromMap(const int &x, const int &y)
    {
        NodeAsPtr node = std::make_shared<NodeAs>(x, y);
        node_set_[node->Node2DTag()] = node;
        // if (costmap_2d_->GridCost(x, y) != GridType::FREE)
        // {
        //     next_node->SetOccupy(true);
        // }
        return node;
    }

    NodeAsPtr Astar::InNodeSet(std::string &tag)
    {
        if (node_set_.find(tag) == node_set_.end())
        {
            return nullptr;
        }
        return node_set_[tag];
    }

    NodeAsPtr Astar::GetNodeFromMap(const int &x, const int &y)
    {
        std::string tag = std::to_string(x) + "_" + std::to_string(y);
        NodeAsPtr node = InNodeSet(tag);
        if (node == nullptr)
        {
            node = CreateNodeFromMap(x, y);
        }
        if (costmap_2d_->GridCost(x, y) != GridType::FREE)
        {
            node->SetOccupy(true);
        }
        else
        {
             node->SetOccupy(false);
        }
        return node;
    }

    NodeAsPtr Astar::GetNodeFromMap(const Cord &pos)
    {
        return GetNodeFromMap(pos.x, pos.y);
    }

    void Astar::ResetSearch()
    {
        closed_set_.clear();
        node_set_.clear();
        decltype(open_pq_) null_pq;
        open_pq_.swap(null_pq);
        search_grids_.clear();
    }

    void Astar::Init()
    {
        ResetSearch();
        // NodeAsPtr start_node = GetNodeFromMap(start_grid_);
        // start_node->SetGCost(0);
        // start_node->SetHCost(HeuristicCost(start_node->Cordinate()));
        //open_pq_.push({start_node->Node2DTag(), CalculateKey(start_node)});
    }

    bool Astar::Plan()
    {
        ResetSearch();
        NodeAsPtr start_node = GetNodeFromMap(start_grid_);
        start_node->SetGCost(0);
        open_pq_.push({start_node->Node2DTag(), CalculateKey(start_node)});
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
            NodeAsPtr cur_node = node_set_[cur_node_tag];
            if (cur_node->Occupy())
            {
                return false;
            }
            //加入闭集———加入进去的不一定是最优解
            closed_set_[cur_node_tag] = cur_node;
            //是终点了，可以退出
            if (node_set_[cur_node_tag]->Cordinate() == end_grid_)
            {
                // GeneratePath();
                return true;
            }
            //搜索
            std::vector<Grid2D> search;
            for (int i = 0; i < 8; i++)
            {
                Cord next_cord = cur_node->Cordinate() + dir_[i];
                next_cord.theta = i;
                //看当前栅格点是否有效
                if (!costmap_2d_->Valid(next_cord.x, next_cord.y))
                {
                    continue;
                }
                NodeAsPtr next_node = GetNodeFromMap(next_cord);
                // double step_dist = dir_[i].norm();
                // double next_gcost = cur_node->GCost() + step_dist;
                // if (next_node->Occupy())
                // {
                //     next_gcost = std::numeric_limits<double>::infinity();
                // }
                double next_gcost = cur_node->GCost() + StepCost(cur_node,next_node);
                if (next_gcost < next_node->GCost())
                {
                    next_node->SetGCost(next_gcost);
                    next_node->SetParent(cur_node_tag);
                    open_pq_.push({next_node->Node2DTag(), CalculateKey(next_node)});
                    // 在闭集里面的话，从闭集拿出来
                    if (closed_set_.find(next_node->Node2DTag()) != closed_set_.end())
                    {
                        closed_set_.erase(next_node->Node2DTag());
                    }
                    search.push_back(next_node->Cordinate());
                }
            }
            AddSearchGrid(search);
        }
        return false;
    }

    double Astar::StepCost(NodeAsPtr &start_node, NodeAsPtr &end_node)
    {
        if(start_node->Occupy() || end_node->Occupy())
        {
            return std::numeric_limits<double>::infinity();
        }
        Cord step = start_node->Cordinate()-end_node->Cordinate();
        return step.norm();
    }

    bool Astar::Plan(Cord &start, Cord &end)
    {
        SetStartGird(start);
        SetEndGird(end);
        return Plan();
    }

    bool Astar::Plan(Pose2D &start, Pose2D &end)
    {
        costmap_2d_->WorldToMap(start.x, start.y, start_grid_.x, start_grid_.y);
        costmap_2d_->WorldToMap(end.x, end.y, end_grid_.x, end_grid_.y);
        return Plan();
    }

    void Astar::GeneratePath(std::vector<Pose2D> &path)
    {
        NodeAsPtr cur = GetNodeFromMap(end_grid_);
        NodeAsPtr start = GetNodeFromMap(start_grid_);
        Pose2D pose;
        while (cur != start)
        {
            costmap_2d_->MapToWorld(cur->Cordinate().x, cur->Cordinate().y, pose.x, pose.y);
            path.push_back(pose);
            std::string tag = cur->Parent();
            cur = node_set_[tag];
        }
        costmap_2d_->MapToWorld(cur->Cordinate().x, cur->Cordinate().y, pose.x, pose.y);
        path.push_back(pose);
        std::reverse(path.begin(), path.end());
    }

} // namespace planner