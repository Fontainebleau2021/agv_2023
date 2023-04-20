#ifndef HYBRID_D_STAR_LITE_H
#define HYBRID_D_STAR_LITE_H

#include "global_planner.h"
#include "node.h"

#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace navigation
{

    // using DstarKey = std::
    using HDsKey = std::pair<double, double>;
    using HDsTagKey = std::pair<std::string, HDsKey>;
    class greathds
    {
    public:
        bool operator()(const HDsTagKey &n1, const HDsTagKey &n2)
        {
            return n1.second.first == n2.second.first ? n1.second.second > n2.second.second : n1.second.first > n2.second.first;
        }
    };

    class HybridDStarLite : public GlobalPlanner
    {

    public:
        HybridDStarLite(/* args */) = default;
        ~HybridDStarLite() {}

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

        //欧式距离启发函数
        double HeuristicCost(const Pose2D &pos)
        {
            Pose2D delta = end_pose_ - pos;
            return (delta).norm();
        }

        void SetStartGird(Cord &start) { start_grid_ = start; }
        void SetEndGird(Cord &end) { end_grid_ = end; }
        void UpdateVertex(NodeHDsPtr node);

        HDsKey CalculateKey(NodeHDsPtr &node)
        {
            double min_g = std::min(node->GCost(), node->rhsCost());
            return {min_g + HeuristicCost(node->NodePose()), min_g};
        }

        virtual void GeneratePath(std::vector<Pose2D> &path);

        void UpdateEdgeCost();

        double StepCost(NodeHDsPtr &start_node, NodeHDsPtr &end_node);

        void GenNextPoses(NodeHDsPtr &cur_node, std::vector<Pose2D> &next_poses, Direction direction);
        void VehicleKinematics(const Pose2D &pose, Pose2D &new_pose, double dis, double theta);
        void NormalizeTheta(double &theta);
    private:
        Pose2D end_pose_;
        Pose2D start_pose_;
        Cord end_grid_;
        Cord start_grid_;
        bool initialized_ = false;
        NodeHDsPtr end_node_;
        NodeHDsPtr start_node_;
        std::unordered_map<std::string, NodeHDsPtr> node_set_;

        std::priority_queue<HDsTagKey, std::vector<HDsTagKey>, greathds> open_pq_;
        std::unordered_map<std::string, NodeHDsPtr> closed_set_;
        std::unordered_set<std::string> open_set_;
        double phi_resolution_ = 0.02;
        double step_dis_ = 0.1; // m 每一步搜索的距离。和resolution尽量相同。

        // error tolerance
        double goal_xy_tolerant_ = 0.1;
        double goal_theta_tolerant_ = 0.1;

        int step_num_ = 3;         //每一次搜索的步数。
        double next_node_num_ = 3; //每一步得到的子节点数。
        double penalty = 0.5;
        double max_angle_ = 0.04; // rad

        /* data */
    };

} // namespace planner

#endif // HYBRID_D_STAR_LITE_H