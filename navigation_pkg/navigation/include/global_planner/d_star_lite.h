#ifndef D_STAR_LITE_H
#define D_STAR_LITE_H

#include "global_planner.h"
#include "node.h"

#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace navigation
{

    // using DstarKey = std::
    using DsKey = std::pair<double, double>;
    using DsTagKey = std::pair<std::string, DsKey>;
    class greatds
    {
    public:
        bool operator()(const DsTagKey &n1, const DsTagKey &n2)
        {
            return n1.second.first == n2.second.first ? n1.second.second > n2.second.second : n1.second.first > n2.second.first;
        }
    };

    class DStarLite : public GlobalPlanner
    {

    public:
        DStarLite(/* args */) = default;
        ~DStarLite() {}

        virtual void Init();
        virtual bool Plan();
        virtual bool Plan(Cord &start, Cord &end);
        virtual bool Plan(Pose2D &start, Pose2D &end);
        NodeDsPtr CreateNodeFromMap(const int &x, const int &y);

        NodeDsPtr CreateNodeFromMap(const Cord &pos);

        NodeDsPtr InNodeSet(std::string &tag);

        NodeDsPtr GetNodeFromMap(const int &x, const int &y);

        NodeDsPtr GetNodeFromMap(const Cord &pos);

        //欧式距离启发函数
        double HeuristicCost(const Cord &pos)
        {
            Cord delta = end_grid_ - pos;
            return (delta).norm();
        }
        void SetStartGird(Cord &start) { start_grid_ = start; }
        void SetEndGird(Cord &end) { end_grid_ = end; }
        void UpdateVertex(NodeDsPtr node);

        DsKey CalculateKey(NodeDsPtr &node)
        {
            double min_g = std::min(node->GCost(), node->rhsCost());
            return {min_g + HeuristicCost(node->Cordinate()), min_g};
        }

        virtual void GeneratePath(std::vector<Pose2D> &path);

        void UpdateEdgeCost();

        double StepCost(NodeDsPtr &start_node, NodeDsPtr &end_node);

        void ResetSearch();

    private:
        Cord end_grid_;
        Cord start_grid_;
        bool initialized_ = false;

        std::unordered_map<std::string, NodeDsPtr> node_set_;

        std::priority_queue<DsTagKey, std::vector<DsTagKey>, greatds> open_pq_;
        std::unordered_map<std::string, NodeDsPtr> closed_set_;
        std::unordered_set<std::string> open_set_;

        std::vector<Cord> dir_ = {Cord(1, 0),
                                  Cord(1, 1),
                                  Cord(0, 1),
                                  Cord(-1, 1),
                                  Cord(-1, 0),
                                  Cord(-1, -1),
                                  Cord(0, -1),
                                  Cord(1, -1)};

        /* data */
    };

} // namespace planner

#endif // D_STAR_LITE_H