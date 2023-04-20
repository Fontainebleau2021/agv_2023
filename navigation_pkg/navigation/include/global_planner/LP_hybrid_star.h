#ifndef LP_HYBRID_A_STAR_H
#define LP_HYBRID_A_STAR_H

#include "global_planner.h"
#include "node.h"

#include <queue>
#include <unordered_map>
#include <vector>
#include <unordered_set>

namespace navigation
{
    using LPHAsKey = std::pair<double, double>;
    using LPHAsTagKey = std::pair<std::string, LPHAsKey>;
    class greatlphas
    {
    public:
        bool operator()(const LPHAsTagKey &n1, const LPHAsTagKey &n2)
        {
            return n1.second.first == n2.second.first ? n1.second.second > n2.second.second : n1.second.first > n2.second.first;
        }
    };

    class LPHAstar : public GlobalPlanner
    {

    public:
        LPHAstar(/* args */) = default;
        ~LPHAstar() {}

        virtual void Init();

        virtual bool Plan();
        virtual bool Plan(Cord &start, Cord &end);
        virtual bool Plan(Pose2D &start, Pose2D &end);

        NodeHDsPtr CreateNodeFromMap(const int &x, const int &y, const int &theta);

        NodeHDsPtr CreateNodeFromMap(const Cord &cord);

        NodeHDsPtr InNodeSet(const std::string &tag);

        NodeHDsPtr GetNodeFromMap(const int &x, const int &y, const int &theta);

        NodeHDsPtr GetNodeFromMap(const Cord &cord);

        NodeHDsPtr GetNodeFromWorld(const Pose2D &pose);

        std::string CalculateNode3DTag(const Pose2D &pos);

        //欧式距离启发函数
        double HeuristicCost(const Pose2D &pos)
        {
            Pose2D delta = end_pose_ - pos;
            return (delta).norm();
        }

        void SetStartGird(Cord &start) { start_grid_ = start; }
        void SetEndGird(Cord &end) { end_grid_ = end; }

        void UpdateVertex(NodeHDsPtr &node);

        LPAsKey CalculateKey(NodeHDsPtr &node)
        {
            double min_g = std::min(node->GCost(), node->rhsCost());
            return {min_g + HeuristicCost(node->NodePose()), min_g};
        }

        virtual void GeneratePath(std::vector<Pose2D> &path);

        void UpdateEdgeCost();

        double StepCost(NodeHDsPtr &start_node, NodeHDsPtr &end_node);

        //限制theta处于【-pi pi】之间
        void NormalizeTheta(double &theta);

        void ResetSearch();

    private:
        Pose2D end_pose_;
        Pose2D start_pose_;
        Cord end_grid_;
        Cord start_grid_;
        bool initialized_ = false;
        std::unordered_map<std::string, NodeHDsPtr> node_set_;

        std::priority_queue<LPHAsTagKey, std::vector<LPHAsTagKey>, greatlphas> open_pq_;
        std::unordered_map<std::string, NodeHDsPtr> closed_set_;
        std::unordered_set<std::string> open_set_;
        /* data */
        double phi_resolution_ = 0.02;
        double step_dis_ = 0.1; // m 每一步搜索的距离。和resolution尽量相同。

        // error tolerance
        double goal_xy_tolerant_ = 0.1;
        double goal_theta_tolerant_ = 0.1;

        int step_num_ = 3;         //每一次搜索的步数。
        double next_node_num_ = 3; //每一步得到的子节点数。
        double penalty = 0.5;
        double max_angle_ = 0.04; // rad
    };
} // namespace navigation

#endif // HYBRID_D_STAR_H