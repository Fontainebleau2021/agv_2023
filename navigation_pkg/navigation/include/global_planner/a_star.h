#ifndef PLANNER_A_STAR_H
#define PLANNER_A_STAR_H

#include "global_planner.h"
#include "node.h"

#include <queue>
#include <unordered_map>
#include <vector>

namespace navigation
{

    class greatas
    {
    public:
        bool operator()(const TagKey &n1, const TagKey &n2)
        {
            return n1.second > n2.second;
        }
    };

    class Astar : public GlobalPlanner
    {

    public:
        Astar(/* args */) = default;
        // Astar() {}

        ~Astar() {}

        virtual void Init();
        virtual bool Plan();
        virtual bool Plan(Cord &start, Cord &end);
        virtual bool Plan(Pose2D &start, Pose2D &end);

        NodeAsPtr CreateNodeFromMap(const int &x, const int &y);
        NodeAsPtr CreateNodeFromMap(const Cord &pos);

        NodeAsPtr InNodeSet(std::string &tag);

        NodeAsPtr GetNodeFromMap(const int &x, const int &y);
        NodeAsPtr GetNodeFromMap(const Cord &pos);

        //欧式距离启发函数
        double HeuristicCost(const Cord &pos)
        {
            Cord delta = end_grid_ - pos;
            double dia = std::min(abs(delta.x), abs(delta.y));
            double stra = abs(delta.x)+abs(delta.y);
            double heu = sqrt(2)*dia + stra-2*dia;
            heu = stra;
            //heu = delta.norm();
            return heu;

        }

        void SetStartGird(Cord &start) { start_grid_ = start; }
        void SetEndGird(Cord &end) { end_grid_ = end; }

        virtual void GeneratePath(std::vector<Pose2D> &path);

        void ResetSearch();

        double CalculateKey(NodeAsPtr &node)
        {
            return (node->GCost() + HeuristicCost(node->Cordinate()));
        }

        double StepCost(NodeAsPtr &start_node, NodeAsPtr &end_node);

    private:
        std::unordered_map<std::string, NodeAsPtr> node_set_;
        Cord end_grid_;
        Cord start_grid_;
        std::priority_queue<TagKey, std::vector<TagKey>, greatas> open_pq_;
        std::unordered_map<std::string, NodeAsPtr> closed_set_;
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

#endif // PLANNER_A_STAR_H