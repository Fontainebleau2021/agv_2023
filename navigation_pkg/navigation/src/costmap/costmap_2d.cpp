
#include "costmap.h"
#include "grid.h"

#include <cstdio>
#include <string.h>
#include <algorithm>
using namespace std;

namespace navigation
{
  Costmap2D::Costmap2D(int cells_size_x, int cells_size_y, double x_resolution,
                       double y_resolution, double origin_x, double origin_y,
                       double inflation_radius, unsigned char default_value)
      : size_x_(cells_size_x), size_y_(cells_size_y), x_resolution_(x_resolution),
        y_resolution_(y_resolution), origin_x_(origin_x), inflation_radius_(inflation_radius),
        origin_y_(origin_y), costmap_(NULL), default_value_(default_value)
  {
    // create the costmap
    inflation_cell_size_x_ = (int)(inflation_radius_ / x_resolution_);
    inflation_cell_size_y_ = (int)(inflation_radius_ / y_resolution_);
    InitCostmap(size_x_, size_y_);
    ResetCostmap();
  }

  Costmap2D &Costmap2D::operator=(const Costmap2D &map)
  {
    // check for self assignement
    if (this == &map)
      return *this;

    // clean up old data
    DeleteCostmap();
    size_x_ = map.size_x_;
    size_y_ = map.size_y_;
    x_resolution_ = map.x_resolution_;
    y_resolution_ = map.y_resolution_;
    origin_x_ = map.origin_x_;
    origin_y_ = map.origin_y_;

    // initialize our various maps
    InitCostmap(size_x_, size_y_);
    // copy the cost map
    memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));
    return *this;
  }
  
  Costmap2D::Costmap2D(const Costmap2D &map) : costmap_(NULL)
  {
    *this = map;
  }

  // just initialize everything to NULL by default
  Costmap2D::Costmap2D() : size_x_(0), size_y_(0), x_resolution_(0.06), y_resolution_(0.1), origin_x_(0.0), origin_y_(0.0), costmap_(NULL)
  {
  }

  Costmap2D::~Costmap2D()
  {
    DeleteCostmap();
  }

  void Costmap2D::InitCostmap(int size_x, int size_y)
  {
    delete[] costmap_;
    costmap_ = new unsigned char[size_x * size_y];
  }

  void Costmap2D::ResetCostmap()
  {
    memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
  }

  void Costmap2D::ResizeCostmap(int size_x, int size_y, double x_resolution,
                                double y_resolution, double origin_x, double origin_y)
  {
    size_x_ = size_x;
    size_y_ = size_y;
    x_resolution_ = x_resolution;
    y_resolution_ = y_resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    InitCostmap(size_x, size_y);
    ResetCostmap();
  }

  void Costmap2D::DeleteCostmap()
  {
    delete[] costmap_;
    costmap_ = NULL;
  }

  

  void Costmap2D::SetGridCost(int mx, int my, unsigned char cost)
  {
    costmap_[getIndex(mx, my)] = cost;
  }

  unsigned char Costmap2D::GridCost(int mx, int my) const
  {
    return costmap_[getIndex(mx, my)];
  }

  void Costmap2D::MapToWorld(int mx, int my, double &wx, double &wy) const
  {
    wx = origin_x_ + (mx + 0.5) * x_resolution_;
    wy = origin_y_ + (my + 0.5) * y_resolution_;
  }

  bool Costmap2D::WorldToMap(double wx, double wy, int &mx, int &my) const
  {
    if (wx < origin_x_ || wy < origin_y_)
      return false;

    mx = static_cast<int>((wx - origin_x_) / x_resolution_);
    my = static_cast<int>((wy - origin_y_) / y_resolution_);
    if (mx < size_x_ && my < size_y_)
      return true;

    return false;
  }

  void Costmap2D::InflationCell(int cell_x, int cell_y)
  {

    for (int i = cell_x - inflation_cell_size_x_; i <= cell_x + inflation_cell_size_x_; i++)
    {
      for (int j = cell_y - inflation_cell_size_y_; j <= cell_y + inflation_cell_size_y_; j++)
      {
        if (Valid(i, j))
        {
          if (GridCost(i, j) == GridType::FREE)
          {
            // printf("inout");
            SetGridCost(i, j, GridType::INFLATION);
          }
        }
      }
    }
  }

 

  void Costmap2D::Inflation()
  {
    for (int i = 0; i < size_x_; i++)
    {
      for (int j = 0; j < size_y_; j++)
      {
        if (GridCost(i, j) == GridType::OBSTACLE)
        {
          // printf("test");
          InflationCell(i, j);
        }
      }
    }
  }

} // namespace navigation
