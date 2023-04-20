#include "costmap_convert.h"

namespace
{

  /**
   * @brief Douglas-Peucker Algorithm for fitting lines into ordered set of points
   *
   * Douglas-Peucker Algorithm, see https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
   *
   * @param begin iterator pointing to the begin of the range of points
   * @param end interator pointing to the end of the range of points
   * @param epsilon distance criteria for removing points if it is closer to the line segment than this
   * @param result the simplified polygon
   */
  std::vector<geometry_msgs::Point32> douglasPeucker(std::vector<geometry_msgs::Point32>::iterator begin,
  std::vector<geometry_msgs::Point32>::iterator end, double epsilon)
{
  if (std::distance(begin, end) <= 2)
  {
    return std::vector<geometry_msgs::Point32>(begin, end);
  }

  // Find the point with the maximum distance from the line [begin, end)
  double dmax = std::numeric_limits<double>::lowest();
  std::vector<geometry_msgs::Point32>::iterator max_dist_it;
  std::vector<geometry_msgs::Point32>::iterator last = std::prev(end);
  for (auto it = std::next(begin); it != last; ++it)
  {
    double d = computeSquaredDistanceToLineSegment(*it, *begin, *last);
    if (d > dmax)
    {
      max_dist_it = it;
      dmax = d;
    }
  }

  if (dmax < epsilon * epsilon)
  { // termination criterion reached, line is good enough
    std::vector<geometry_msgs::Point32> result;
    result.push_back(*begin);
    result.push_back(*last);
    return result;
  }

  // Recursive calls for the two splitted parts
  auto firstLineSimplified = douglasPeucker(begin, std::next(max_dist_it), epsilon);
  auto secondLineSimplified = douglasPeucker(max_dist_it, end, epsilon);

  // Combine the two lines into one line and return the merged line.
  // Note that we have to skip the first point of the second line, as it is duplicated above.
  firstLineSimplified.insert(firstLineSimplified.end(),
    std::make_move_iterator(std::next(secondLineSimplified.begin())),
    std::make_move_iterator(secondLineSimplified.end()));
  return firstLineSimplified;
}

} // end namespace

namespace navigation
{
  void CostmapConvert::updateCostmap2D(Costmap2DPtr& costmap)
  {
    occupied_cells_.clear();

    // allocate neighbor lookup
    int cells_x = int(costmap->getSizeInMetersX() / parameter_.max_distance_) + 1;
    int cells_y = int(costmap->getSizeInMetersY() / parameter_.max_distance_) + 1;

    if (cells_x != neighbor_size_x_ || cells_y != neighbor_size_y_)
    {
      neighbor_size_x_ = cells_x;
      neighbor_size_y_ = cells_y;
      neighbor_lookup_.resize(neighbor_size_x_ * neighbor_size_y_);
    }
    offset_x_ = costmap->OriginX();
    offset_y_ = costmap->OriginY();
    for (auto &n : neighbor_lookup_)
      n.clear();

    // get indices of obstacle cells
    for (std::size_t i = 0; i < costmap->SizeX(); i++)
    {
      for (std::size_t j = 0; j < costmap->SizeY(); j++)
      {
        int value = costmap->GridCost(i, j);
        if (value == GridType::OBSTACLE)
        {
          double x, y;
          costmap->MapToWorld((unsigned int)i, (unsigned int)j, x, y);
          addPoint(x, y);
        }
      }
    }
  }

  void CostmapConvert::compute()
  {
    std::vector<std::vector<KeyPoint>> clusters;
    dbScan(clusters);

    // Create new polygon container
    PolygonContainerPtr polygons(new std::vector<geometry_msgs::Polygon>());

    // add convex hulls to polygon container
    for (std::size_t i = 1; i < clusters.size(); ++i) // skip first cluster, since it is just noise
    {
      polygons->push_back( geometry_msgs::Polygon() );
      convexHull2(clusters[i], polygons->back());
    }

    // add our non-cluster points to the polygon container (as single points)
    if (!clusters.empty())
    {
      for (std::size_t i = 0; i < clusters.front().size(); ++i)
      {
        polygons->push_back( geometry_msgs::Polygon() );
        convertPointToPolygon(clusters.front()[i], polygons->back());
      }
    }

    // replace shared polygon container
    updatePolygonContainer(polygons);
  }

  void CostmapConvert::updatePolygonContainer(PolygonContainerPtr polygons)
  {
    // boost::mutex::scoped_lock lock(mutex_);
    polygons_ = polygons;
  }

  void CostmapConvert::dbScan(std::vector<std::vector<KeyPoint>> &clusters)
  {
    std::vector<bool> visited(occupied_cells_.size(), false);

    clusters.clear();

    // DB Scan Algorithm
    int cluster_id = 0; // current cluster_id
    clusters.push_back(std::vector<KeyPoint>());
    for (int i = 0; i < (int)occupied_cells_.size(); i++)
    {
      if (!visited[i]) // keypoint has not been visited before
      {
        visited[i] = true; // mark as visited
        std::vector<int> neighbors;
        regionQuery(i, neighbors);                       // Find neighbors around the keypoint
        if ((int)neighbors.size() < parameter_.min_pts_) // If not enough neighbors are found, mark as noise
        {
          clusters[0].push_back(occupied_cells_[i]);
        }
        else
        {
          ++cluster_id; // increment current cluster_id
          clusters.push_back(std::vector<KeyPoint>());

          // Expand the cluster
          clusters[cluster_id].push_back(occupied_cells_[i]);
          for (int j = 0; j < (int)neighbors.size(); j++)
          {
            if ((int)clusters[cluster_id].size() == parameter_.max_pts_)
              break;

            if (!visited[neighbors[j]]) // keypoint has not been visited before
            {
              visited[neighbors[j]] = true; // mark as visited
              std::vector<int> further_neighbors;
              regionQuery(neighbors[j], further_neighbors); // Find more neighbors around the new keypoint
              //             if(further_neighbors.size() < min_pts_)
              //             {
              //               clusters[0].push_back(occupied_cells[neighbors[j]]);
              //             }
              //             else
              if ((int)further_neighbors.size() >= parameter_.min_pts_)
              {
                // neighbors found
                neighbors.insert(neighbors.end(), further_neighbors.begin(), further_neighbors.end()); // Add these newfound P' neighbour to P neighbour vector "nb_indeces"
                clusters[cluster_id].push_back(occupied_cells_[neighbors[j]]);
              }
            }
          }
        }
      }
    }
  }

  void CostmapConvert::regionQuery(int curr_index, std::vector<int> &neighbors)
  {
    neighbors.clear();

    double dist_sqr_threshold = parameter_.max_distance_ * parameter_.max_distance_;
    const KeyPoint &kp = occupied_cells_[curr_index];
    int cx, cy;
    pointToNeighborCells(kp, cx, cy);

    // loop over the neighboring cells for looking up the points
    const int offsets[9][2] = {{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {0, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}};
    for (int i = 0; i < 9; ++i)
    {
      int idx = neighborCellsToIndex(cx + offsets[i][0], cy + offsets[i][1]);
      if (idx < 0 || idx >= int(neighbor_lookup_.size()))
        continue;
      const std::vector<int> &pointIndicesToCheck = neighbor_lookup_[idx];
      for (int point_idx : pointIndicesToCheck)
      {
        if (point_idx == curr_index) // point is not a neighbor to itself
          continue;
        const KeyPoint &other = occupied_cells_[point_idx];
        double dx = other.x - kp.x;
        double dy = other.y - kp.y;
        double dist_sqr = dx * dx + dy * dy;
        if (dist_sqr <= dist_sqr_threshold)
          neighbors.push_back(point_idx);
      }
    }
  }
  bool isXCoordinateSmaller(const KeyPoint &p1, const KeyPoint &p2)
  {
    return p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y);
  }
  void CostmapConvert::convexHull2(std::vector<KeyPoint>& cluster, geometry_msgs::Polygon& polygon)
  {
    std::vector<KeyPoint> &P = cluster;
    std::vector<geometry_msgs::Point32> &points = polygon.points;

    // Sort P by x and y
    std::sort(P.begin(), P.end(), isXCoordinateSmaller);

    // the output array H[] will be used as the stack
    int i; // array scan index

    // Get the indices of points with min x-coord and min|max y-coord
    int minmin = 0, minmax;
    double xmin = P[0].x;
    for (i = 1; i < (int)P.size(); i++)
      if (P[i].x != xmin)
        break;
    minmax = i - 1;
    if (minmax == (int)P.size() - 1)
    { // degenerate case: all x-coords == xmin
      points.push_back(geometry_msgs::Point32());
      P[minmin].toPointMsg(points.back());
      if (P[minmax].y != P[minmin].y) // a  nontrivial segment
      {
        points.push_back(geometry_msgs::Point32());
        P[minmax].toPointMsg(points.back());
      }
      // add polygon endpoint
      points.push_back(geometry_msgs::Point32());
      P[minmin].toPointMsg(points.back());
      return;
    }

    // Get the indices of points with max x-coord and min|max y-coord
    int maxmin, maxmax = (int)P.size() - 1;
    double xmax = P.back().x;
    for (i = P.size() - 2; i >= 0; i--)
      if (P[i].x != xmax)
        break;
    maxmin = i + 1;

    // Compute the lower hull on the stack H
    // push  minmin point onto stack
    points.push_back(geometry_msgs::Point32());
    P[minmin].toPointMsg(points.back());
    i = minmax;
    while (++i <= maxmin)
    {
      // the lower line joins P[minmin]  with P[maxmin]
      if (cross(P[minmin], P[maxmin], P[i]) >= 0 && i < maxmin)
        continue; // ignore P[i] above or on the lower line

      while (points.size() > 1) // there are at least 2 points on the stack
      {
        // test if  P[i] is left of the line at the stack top
        if (cross(points[points.size() - 2], points.back(), P[i]) > 0)
          break;           // P[i] is a new hull  vertex
        points.pop_back(); // pop top point off  stack
      }
      // push P[i] onto stack
      points.push_back(geometry_msgs::Point32());
      P[i].toPointMsg(points.back());
    }

    // Next, compute the upper hull on the stack H above  the bottom hull
    if (maxmax != maxmin) // if  distinct xmax points
    {
      // push maxmax point onto stack
      points.push_back(geometry_msgs::Point32());
      P[maxmax].toPointMsg(points.back());
    }
    int bot = (int)points.size(); // the bottom point of the upper hull stack
    i = maxmin;
    while (--i >= minmax)
    {
      // the upper line joins P[maxmax]  with P[minmax]
      if (cross(P[maxmax], P[minmax], P[i]) >= 0 && i > minmax)
        continue; // ignore P[i] below or on the upper line

      while ((int)points.size() > bot) // at least 2 points on the upper stack
      {
        // test if  P[i] is left of the line at the stack top
        if (cross(points[points.size() - 2], points.back(), P[i]) > 0)
          break;           // P[i] is a new hull  vertex
        points.pop_back(); // pop top point off stack
      }
      // push P[i] onto stack
      points.push_back(geometry_msgs::Point32());
      P[i].toPointMsg(points.back());
    }
    if (minmax != minmin)
    {
      // push  joining endpoint onto stack
      points.push_back(geometry_msgs::Point32());
      P[minmin].toPointMsg(points.back());
    }

    simplifyPolygon(polygon);
  }

  void CostmapConvert::simplifyPolygon(geometry_msgs::Polygon& polygon)
  {
    size_t triangleThreshold = 3;
    // check if first and last point are the same. If yes, a triangle has 4 points
    if (polygon.points.size() > 1 && std::abs(polygon.points.front().x - polygon.points.back().x) < 1e-5 && std::abs(polygon.points.front().y - polygon.points.back().y) < 1e-5)
    {
      triangleThreshold = 4;
    }
    if (polygon.points.size() <= triangleThreshold) // nothing to do for triangles or lines
      return;
    // TODO Reason about better start conditions for splitting lines, e.g., by
    // https://en.wikipedia.org/wiki/Rotating_calipers
    polygon.points = douglasPeucker(polygon.points.begin(), polygon.points.end(), parameter_.min_keypoint_separation_);
    ;
  }

} // namespace navigation
