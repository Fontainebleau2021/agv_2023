#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H

#include "global_planner.h"
#include "node.h"

#include <queue>
#include <unordered_map>
#include <vector>

namespace navigation
{

    class greathas
    {
    public:
        bool operator()(const TagKey &n1, const TagKey &n2)
        {
            return n1.second > n2.second;
        }
    };

    class HybridAstar : public GlobalPlanner
    {
    public:
        HybridAstar(/* args */) = default;
        ~HybridAstar() {}

        virtual bool Plan();
        virtual bool Plan(Cord &start, Cord &end);
        virtual bool Plan(Pose2D &start, Pose2D &end);

        NodeHAsPtr CreateNodeFromMap(const int &x, const int &y, const int &theta);

        NodeHAsPtr CreateNodeFromMap(const Cord &cord);

        NodeHAsPtr InNodeSet(const std::string &tag);

        NodeHAsPtr GetNodeFromMap(const int &x, const int &y, const int &theta);

        NodeHAsPtr GetNodeFromMap(const Cord &cord);

        NodeHAsPtr GetNodeFromWorld(const Pose2D &pose);

        std::string CalculateNode3DTag(const Pose2D &pos);

        //欧式距离启发函数
        double HeuristicCost(const Pose2D &pos)
        {
            Pose2D delta = end_pose_ - pos;
            return (delta).norm();
        }

        double CalculateKey(const NodeHAsPtr &node)
        {
            return node->GCost() + HeuristicCost(node->NodePose());
        }

        void SetStartGird(Cord &start) { start_grid_ = start; }
        void SetEndGird(Cord &end) { end_grid_ = end; }

        void GenNextNodes(NodeHAsPtr &cur_node, std::vector<Pose2D> &next_poses, Direction direction);

        //机器人以当前位置为圆心画半径为dis的园，当前朝向为正朝向，旋转theta角度后与圆的交点为下一个节点
        void VehicleKinematics(const Pose2D &pose, Pose2D &new_pose, double dis, double theta);

        bool IsArriveTarget(NodeHAsPtr &cur_node);

        void ResetSearch();
        virtual void Init();

        virtual void GeneratePath(std::vector<Pose2D> &path);

        //限制theta处于【-pi pi】之间
        void NormalizeTheta(double &theta);

        double StepCost(NodeHAsPtr &start_node, NodeHAsPtr &end_node);

    private:
        Pose2D end_pose_;
        Pose2D start_pose_;
        Cord end_grid_;
        Cord start_grid_;
        std::unordered_map<std::string, NodeHAsPtr> node_set_;
        std::priority_queue<TagKey, std::vector<TagKey>, greathas> open_pq_;
        std::unordered_map<std::string, NodeHAsPtr> closed_set_;

        double phi_resolution_ = 0.02;
        double step_dis_ = 0.1; // m 每一步搜索的距离。和resolution尽量相同。

        // error tolerance
        double goal_xy_tolerant_ = 0.06;
        double goal_theta_tolerant_ = 0.05;

        int step_num_ = 3;         //每一次搜索的步数。
        double next_node_num_ = 3; //每一步得到的子节点数。
        double penalty = 0.5;
        double max_angle_ = 0.03; // rad
    };
} // namespace navigation

#endif