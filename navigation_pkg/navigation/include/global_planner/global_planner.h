#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include "costmap.h"
#include "node.h"
#include "pose2D.h"

#include <memory>
#include <vector>

namespace navigation
{
    using TagKey = std::pair<std::string, double>;
    class GlobalPlanner
    {
    public:
        GlobalPlanner() = default;

        virtual ~GlobalPlanner() {}

        virtual bool Plan() = 0;

        virtual bool Plan(Cord &start, Cord &end) = 0;
        virtual bool Plan(Pose2D &start, Pose2D &end) = 0;

        virtual void GeneratePath(std::vector<Pose2D> &path) = 0;

        void SetCostmap2d(Costmap2DPtr &costmap_2d) { costmap_2d_ = costmap_2d; }
        virtual void Init() = 0;
        void AddSearchGrid(std::vector<Grid2D> &grid)
        {
            search_grids_.push_back(grid);
        }
        std::vector<std::vector<Grid2D>> SearchGrid() {  return search_grids_; }

    protected:
        Costmap2DPtr costmap_2d_;
        std::vector<std::vector<Grid2D>> search_grids_;
    };

    typedef std::shared_ptr<GlobalPlanner> GlobalPlannerPtr;

} // namespace planner

#endif // GLOBAL_PLANNER_H