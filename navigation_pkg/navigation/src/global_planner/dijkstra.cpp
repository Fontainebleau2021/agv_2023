#include "dijkstra.h"

namespace navigation
{
     
    Node2DPtr Dijkstra::CreateNodeFromMap(const Cord &pos)
    {
        return CreateNodeFromMap(pos.x, pos.y);
    }

    Node2DPtr Dijkstra::CreateNodeFromMap(const int &x, const int &y)
    {
        Node2DPtr node = std::make_shared<Node2D>(x, y);
        node_set_[node->Node2DTag()] = node;    
        return node;
    }

    Node2DPtr Dijkstra::InNodeSet(std::string &tag)
    {
        if (node_set_.find(tag) == node_set_.end())
        {
            return nullptr;
        }
        return node_set_[tag];
    }

    Node2DPtr Dijkstra::GetNodeFromMap(const int &x, const int &y)
    {
        std::string tag = std::to_string(x) + "_" + std::to_string(y);
        Node2DPtr node = InNodeSet(tag);
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

    Node2DPtr Dijkstra::GetNodeFromMap(const Cord &pos)
    {
        return GetNodeFromMap(pos.x, pos.y);
    }

    void Dijkstra::ResetSearch()
    {
        closed_set_.clear();
        node_set_.clear();
        decltype(open_pq_) null_pq;
        open_pq_.swap(null_pq);
        search_grids_.clear();
    }

    void Dijkstra::Init()
    {
        ResetSearch();
        // NodeAsPtr start_node = GetNodeFromMap(start_grid_);
        // start_node->SetGCost(0);
        // start_node->SetHCost(HeuristicCost(start_node->Cordinate()));
        //open_pq_.push({start_node->Node2DTag(), CalculateKey(start_node)});
    }

    bool Dijkstra::Plan()
    {
        ResetSearch();
        Node2DPtr start_node = GetNodeFromMap(start_grid_);
        start_node->SetGCost(0);
        open_pq_.push({start_node->Node2DTag(), start_node->GCost()});
        while (!open_pq_.empty())
        {
            TagKey cur_tagkey = open_pq_.top();
            open_pq_.pop();
            std::string cur_node_tag = cur_tagkey.first;
            // // 在闭集里面，不需要再考虑。
            if (closed_set_.find(cur_node_tag) != closed_set_.end())
            {
                continue;
            }
            //得到当前node
            Node2DPtr cur_node = node_set_[cur_node_tag];
            //加入闭集
            closed_set_[cur_node_tag] = cur_node;
            //是终点了，可以退出
            if (cur_node->Cordinate() == end_grid_)
            {
                //GeneratePath();
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
                Node2DPtr next_node = GetNodeFromMap(next_cord);
                // 访问过的一定是最小的那个，不需要再考虑
                if (closed_set_.find(next_node->Node2DTag()) != closed_set_.end())
                {
                    continue;
                }
                double next_gcost = cur_node->GCost() + StepCost(cur_node,next_node);
                if (next_gcost < next_node->GCost())
                {
                    next_node->SetGCost(next_gcost);
                    next_node->SetParent(cur_node_tag);
                    open_pq_.push({next_node->Node2DTag(), next_node->GCost()});
                    search.push_back(next_node->Cordinate());
                }
            }
            AddSearchGrid(search);
        }
        return false;
    }

    double Dijkstra::StepCost(Node2DPtr &start_node, Node2DPtr &end_node)
    {
        if(start_node->Occupy() || end_node->Occupy())
        {
            return std::numeric_limits<double>::infinity();
        }
        Cord step = start_node->Cordinate()-end_node->Cordinate();
        return step.norm();
    }

    bool Dijkstra::Plan(Cord &start, Cord &end)
    {
        SetStartGird(start);
        SetEndGird(end);
        return Plan();
    }

    bool Dijkstra::Plan(Pose2D &start, Pose2D &end)
    {
        costmap_2d_->WorldToMap(start.x, start.y, start_grid_.x, start_grid_.y);
        costmap_2d_->WorldToMap(end.x, end.y, end_grid_.x, end_grid_.y);
        return Plan();
    }

    void Dijkstra::GeneratePath(std::vector<Pose2D> &path)
    {
        Node2DPtr cur = GetNodeFromMap(end_grid_);
        Node2DPtr start = GetNodeFromMap(start_grid_);
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
