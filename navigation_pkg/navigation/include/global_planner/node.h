#ifndef PLANNER_NODE_H
#define PLANNER_NODE_H

#include "grid.h"
#include "pose2D.h"
#include <string>
#include <limits>
#include <memory>

namespace navigation
{
    using Cord = Grid2D;
    class Node2D
    {
    public:
        Node2D() = default;
        Node2D(const int &x, const int &y)
            : cord_(x, y), occupied_(false)
        {
            node2d_tag_ = std::to_string(x) + "_" + std::to_string(y);
        }

        Node2D(const int &x, const int &y, bool occupy)
            : cord_(x, y), occupied_(occupy)
        {
            node2d_tag_ = std::to_string(x) + "_" + std::to_string(y);
        }

        ~Node2D() {}

        void SetNode2DTag() { node2d_tag_ = std::to_string(cord_.x) + "_" + std::to_string(cord_.y); }
        void SetNode2DTag(const int &x, const int &y)
        {
            cord_.x = x;
            cord_.y = y;
            SetNode2DTag();
        }

        Cord Cordinate() { return cord_; }

        std::string Node2DTag() { return node2d_tag_; }

        void SetParent(const std::string &pa)
        {
            parent_ = pa;
        }
        std::string Parent() { return parent_; }

        void SetGCost(const double &cost) { g_cost_ = cost; }
        double GCost() { return g_cost_; }

        void SetOccupy(bool occ) { occupied_ = occ; }
        bool Occupy() { return occupied_; }

    protected:
        Cord cord_;
        std::string node2d_tag_;
        std::string parent_;
        bool occupied_;
        double g_cost_ = std::numeric_limits<double>::infinity();
    };

    using Node2DPtr = std::shared_ptr<Node2D>;

    class NodeAs : public Node2D
    {
    public:
        NodeAs() = default;
        NodeAs(const int &x, const int &y) : Node2D(x, y) {}
        NodeAs(const int &x, const int &y, bool occupy) : Node2D(x, y, occupy) {}
        void SetHCost(const double &cost) { h_cost_ = cost; }
        double HCost() { return h_cost_; }

    private:
        double h_cost_ = std::numeric_limits<double>::infinity();
    };
    using NodeAsPtr = std::shared_ptr<NodeAs>;

    class NodeDs : public Node2D
    {
    public:
        NodeDs() = default;
        NodeDs(const int &x, const int &y) : Node2D(x, y) {}
        NodeDs(const int &x, const int &y, bool occupy) : Node2D(x, y, occupy) {}

        void SetHcost(const double &cost) { h_cost_ = cost; }
        double HCost() { return h_cost_; }

        void SetrhsCost(const double &cost) { rhs_ = cost; }
        double rhsCost() { return rhs_; }

    private:
        double rhs_ = std::numeric_limits<double>::infinity();
        double h_cost_ = std::numeric_limits<double>::infinity();
    };

    using NodeDsPtr = std::shared_ptr<NodeDs>;

    enum Direction
    {
        Forward = 0,
        Circle = 1,
        Backward = 2,
        None = 3,

    };

    class NodeHAs : public Node2D
    {
    public:
        NodeHAs() = default;
        NodeHAs(const int &x, const int &y, const int &theta) : Node2D(x, y)
        {
            node3d_tag_ = std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(theta);
            cord_.theta = theta;
        }

        void SetHCost(const double &cost) { h_cost_ = cost; }
        double HCost() { return h_cost_; }

        void SetNodePose(const Pose2D &pose) { pose2d_ = pose; }
        Pose2D NodePose() { return pose2d_; }

        std::string NodeHAsTag() { return node3d_tag_; }

        Direction NodeDir() { return direction_; }
        void SetNodeDir(Direction dir) { direction_ = dir; }

    private:
        std::string node3d_tag_;
        double h_cost_ = std::numeric_limits<double>::infinity();
        // double dis_cost_ = std::numeric_limits<double>::infinity();
        
        Direction direction_;
        Pose2D pose2d_;
    };

    using NodeHAsPtr = std::shared_ptr<NodeHAs>;


    class NodeHDs : public Node2D
    {
    public:
        NodeHDs() = default;
        NodeHDs(const int &x, const int &y, const int &theta) : Node2D(x, y)
        {
            node3d_tag_ = std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(theta);
            cord_.theta = theta;
        }

        void SetHCost(const double &cost) { h_cost_ = cost; }
        double HCost() { return h_cost_; }

        void SetNodePose(const Pose2D &pose) { pose2d_ = pose; }
        Pose2D NodePose() { return pose2d_; }

        std::string NodeHDsTag() { return node3d_tag_; }

        Direction NodeDir() { return direction_; }
        void SetNodeDir(Direction dir) { direction_ = dir; }

        void SetrhsCost(const double &cost) { rhs_ = cost; }
        double rhsCost() { return rhs_; }

    private:
        std::string node3d_tag_;
        double h_cost_ = std::numeric_limits<double>::infinity();
        // double dis_cost_ = std::numeric_limits<double>::infinity();
        double rhs_ = std::numeric_limits<double>::infinity();
        Direction direction_;
        Pose2D pose2d_;
    };

    using NodeHDsPtr = std::shared_ptr<NodeHDs>;


} // namespace navigation

#endif // PLANNER_NODE_H