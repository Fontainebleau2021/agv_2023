#ifndef NAVI_POSE2D_H
#define NAVI_POSE2D_H

#include <math.h>

namespace navigation
{
    struct Pose2D
    {
        /* data */
        double x;
        double y;
        double theta;
        Pose2D(double vx, double vy, double vt) : x(vx), y(vy), theta(vt) {}
        Pose2D(double vx, double vy) : x(vx), y(vy), theta(0) {}
        Pose2D() : x(0), y(0), theta(0) {}
        double norm()
        {
            return std::hypot(x, y);
        }

        Pose2D &operator=(const Pose2D &pose)
        {
            // Pose2D p;
            if (this != &pose)
            {
                this->x = pose.x;
                this->y = pose.y;
                this->theta = pose.theta;
            }
            return *this;
        }

        Pose2D operator-(const Pose2D &pose)
        {
            Pose2D p;
            p.x = x - pose.x;
            p.y = y - pose.y;
            p.theta = theta - pose.theta;
            return p;
        }

        Pose2D operator+(const Pose2D &pose)
        {
            Pose2D p;
            p.x = x + pose.x;
            p.y = y + pose.y;
            p.theta = theta + pose.theta;
            return p;
        }
    };
} // namespace navigation




#endif // NAVI_POSE2D_H