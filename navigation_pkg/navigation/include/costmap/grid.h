#ifndef NAVI_GRID_H
#define NAVI_GRID_H

#include "math.h"

namespace navigation
{
    struct Grid2D
    {
        /* data */
        int x;
        int y;
        int theta;
        Grid2D(int vx, int vy, int vt) : x(vx), y(vy), theta(vt) {}
        Grid2D(int vx, int vy) : x(vx), y(vy), theta(0) {}
        Grid2D() : x(0), y(0), theta(0) {}
        double norm()
        {
            return std::hypot(x, y);
        }

        Grid2D &operator=(const Grid2D &grid)
        {
            if (this != &grid)
            {
                this->x = grid.x;
                this->y = grid.y;
                this->theta = grid.theta;
            }
            return *this;
        }

        Grid2D operator-(const Grid2D &grid)
        {
            Grid2D g;
            g.x = x - grid.x;
            g.y = y - grid.y;
            g.theta = theta - grid.theta;
            return g;
        }

        Grid2D operator+(const Grid2D &grid)
        {
            Grid2D g;
            g.x = x + grid.x;
            g.y = y + grid.y;
            g.theta = theta + grid.theta;
            return g;
        }

        bool operator==(const Grid2D &grid)
        {
            if (this->x == grid.x && this->y == grid.y && this->theta == grid.theta)
            {
                return true;
            }
            return false;
        }

        bool operator!=(const Grid2D &grid)
        {
            if (this->x != grid.x || this->y != grid.y)
            {
                return true;
            }
            return false;
        }
    };

    enum GridType
    {
        OBSTACLE = 0,
        FREE = 255,
        INFLATION = 122,
    };
} // namespace navigation

#endif // NAVI_GRID_H