#ifndef PLANNER_DIJKSTRA_H
#define PLANNER_DIJKSTRA_H

#include "global_planner.h"
#include "node.h"

#include <queue>
#include <unordered_map>
#include <vector>

namespace navigation
{

    class great
    {
    public:
        bool operator()(const TagKey &n1, const TagKey &n2)
        {
            return n1.second > n2.second;
        }
    };

    class Dijkstra : public GlobalPlanner
    {

    public:
        Dijkstra(/* args */) = default;  
        ~Dijkstra() {}

        virtual void Init();
        virtual bool Plan();
        virtual bool Plan(Cord &start, Cord &end);
        virtual bool Plan(Pose2D &start, Pose2D &end);

        Node2DPtr CreateNodeFromMap(const int &x, const int &y);
        Node2DPtr CreateNodeFromMap(const Cord &pos);

        Node2DPtr InNodeSet(std::string &tag);

        Node2DPtr GetNodeFromMap(const int &x, const int &y);
        Node2DPtr GetNodeFromMap(const Cord &pos);


        void SetStartGird(Cord &start) { start_grid_ = start; }
        void SetEndGird(Cord &end) { end_grid_ = end; }

        virtual void GeneratePath(std::vector<Pose2D> &path);

        void ResetSearch();

        double CalculateKey(Node2DPtr &node)
        {
            return node->GCost();
        }

        double StepCost(Node2DPtr &start_node, Node2DPtr &end_node);

    private:
        std::unordered_map<std::string, Node2DPtr> node_set_;
        Cord end_grid_;
        Cord start_grid_;
        std::priority_queue<TagKey, std::vector<TagKey>, great> open_pq_;
        std::unordered_map<std::string, Node2DPtr> closed_set_;
        std::vector<Cord> dir_ = {Cord(1, 0),
                                  Cord(1, 1),
                                  Cord(0, 1),
                                  Cord(-1, 1),
                                  Cord(-1, 0),
                                  Cord(-1, -1),
                                  Cord(0, -1),
                                  Cord(1, -1)};
    };

} // namespace planner

#endif // DIJKSTRA_H