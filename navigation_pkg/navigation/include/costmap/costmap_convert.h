#ifndef NAVI_COSTMAP_CONVERT_H
#define NAVI_COSTMAP_CONVERT_H

#include <geometry_msgs/Polygon.h>
#include "costmap.h"
#include "obstacles.h"

#include <memory>

template <typename Point, typename LinePoint>
inline double computeSquaredDistanceToLineSegment(const Point &point, const LinePoint &line_start, const LinePoint &line_end, bool *is_inbetween = NULL)
{
    double dx = line_end.x - line_start.x;
    double dy = line_end.y - line_start.y;

    double length_sqr = dx * dx + dy * dy;

    double u = 0;

    if (length_sqr > 0)
        u = ((point.x - line_start.x) * dx + (point.y - line_start.y) * dy) / length_sqr;

    if (is_inbetween)
        *is_inbetween = (u >= 0 && u <= 1);

    if (u <= 0)
        return std::pow(point.x - line_start.x, 2) + std::pow(point.y - line_start.y, 2);

    if (u >= 1)
        return std::pow(point.x - line_end.x, 2) + std::pow(point.y - line_end.y, 2);

    return std::pow(point.x - (line_start.x + u * dx), 2) + std::pow(point.y - (line_start.y + u * dy), 2);
}

namespace navigation
{

    //! Typedef for a shared polygon container
    typedef boost::shared_ptr<std::vector<geometry_msgs::Polygon>> PolygonContainerPtr;
    //! Typedef for a shared polygon container (read-only access)
    typedef boost::shared_ptr<const std::vector<geometry_msgs::Polygon>> PolygonContainerConstPtr;

    // convenient for storing x/y point pairs

    struct KeyPoint
    {
        //! Default constructor
        KeyPoint() {}
        //! Constructor with point initialization
        KeyPoint(double x_, double y_) : x(x_), y(y_) {}
        double x; //!< x coordinate [m]
        double y; //!< y coordinate [m]

        //! Convert keypoint to geometry_msgs::Point message type
        void toPointMsg(geometry_msgs::Point &point) const
        {
            point.x = x;
            point.y = y;
            point.z = 0;
        }
        //! Convert keypoint to geometry_msgs::Point32 message type
        void toPointMsg(geometry_msgs::Point32 &point) const
        {
            point.x = x;
            point.y = y;
            point.z = 0;
        }
    };

    struct Parameters
    {
        Parameters() : max_distance_(0.4), min_pts_(2), max_pts_(100), min_keypoint_separation_(0.1) {}
        // DBSCAN parameters
        double max_distance_; //!< Parameter for DB_Scan, maximum distance to neighbors [m]
        int min_pts_;         //!< Parameter for DB_Scan: minimum number of points that define a cluster
        int max_pts_;         //!< Parameter for DB_Scan: maximum number of points that define a cluster (to avoid large L- and U-shapes)

        // convex hull parameters
        double min_keypoint_separation_; //!< Clear keypoints of the convex polygon that are close to each other [distance in meters] (0: keep all)
    };

    class CostmapConvert
    {

    public:
        CostmapConvert(/* args */)
        {
            neighbor_size_x_ = neighbor_size_y_ = -1;
            offset_x_ = offset_y_ = 0.;
            parameter_ = Parameters();
        }
        ~CostmapConvert() {}
        void updateCostmap2D(Costmap2DPtr &costmap);

        void compute();

        void dbScan(std::vector<std::vector<KeyPoint>> &clusters);
        void regionQuery(int curr_index, std::vector<int> &neighbors);

        void convexHull2(std::vector<KeyPoint> &cluster, geometry_msgs::Polygon &polygon);
        void simplifyPolygon(geometry_msgs::Polygon &polygon);
        void updatePolygonContainer(PolygonContainerPtr polygons);
        void pointToNeighborCells(const KeyPoint &kp, int &cx, int &cy)
        {
            cx = int((kp.x - offset_x_) / parameter_.max_distance_);
            cy = int((kp.y - offset_y_) / parameter_.max_distance_);
        }

        int neighborCellsToIndex(int cx, int cy)
        {
            if (cx < 0 || cx >= neighbor_size_x_ || cy < 0 || cy >= neighbor_size_y_)
                return -1;
            return cy * neighbor_size_x_ + cx;
        }

        template <typename P1, typename P2, typename P3>
        long double cross(const P1 &O, const P2 &A, const P3 &B)
        {
            return (long double)(A.x - O.x) * (long double)(B.y - O.y) - (long double)(A.y - O.y) * (long double)(B.x - O.x);
        }

        template <typename Point>
        static void convertPointToPolygon(const Point &point, geometry_msgs::Polygon &polygon)
        {
            polygon.points.resize(1);
            polygon.points.front().x = point.x;
            polygon.points.front().y = point.y;
            polygon.points.front().z = 0;
        }

        void addPoint(double x, double y)
        {
            int idx = occupied_cells_.size();
            occupied_cells_.emplace_back(x, y);
            int cx, cy;
            pointToNeighborCells(occupied_cells_.back(), cx, cy);
            int nidx = neighborCellsToIndex(cx, cy);
            if (nidx >= 0)
                neighbor_lookup_[nidx].push_back(idx);
        }

        PolygonContainerConstPtr Polygons() { return polygons_; }

        void UpdateObstaclePtr(std::vector<ObstaclePtr> &obst_vector)
        {

            for (int i = 0; i < polygons_->size(); i++)
            {
                Point2dContainer obs_points;
                for (int j = 0; j < polygons_->at(i).points.size(); j++)
                {
                    obs_points.push_back(Eigen::Vector2d(polygons_->at(i).points[j].x, polygons_->at(i).points[j].y));
                }
                auto po_obs = boost::make_shared<PolygonObstacle>(obs_points);
                po_obs->finalizePolygon();
                obst_vector.push_back(po_obs);
            }
        }

    private:
        /* data */
        int neighbor_size_x_; //! size of the neighbour lookup in x (number of cells)
        int neighbor_size_y_;
        std::vector<KeyPoint> occupied_cells_; //!< List of occupied cells in the current map (updated by updateCostmap2D())
        std::vector<std::vector<int>> neighbor_lookup_;
        Parameters parameter_;
        PolygonContainerPtr polygons_;
        double offset_x_; //! offset [meters] in x for the lookup grid
        double offset_y_;
    };

} // namespace navigation

#endif