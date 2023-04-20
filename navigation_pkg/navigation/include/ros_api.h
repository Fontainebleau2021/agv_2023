#ifndef NAVI_ROS_API_H
#define NAVI_ROS_API_H

#include <ros/ros.h>

namespace navigation
{
    template <typename T>
    class RosPublisher
    {
    public:
        RosPublisher(ros::NodeHandle *nh, std::string topic_name) : nh_(nh), topic_name_(topic_name), queue_size_(10)
        {
            Init();
        }
        RosPublisher(ros::NodeHandle *nh, std::string topic_name, int size) : nh_(nh), topic_name_(topic_name), queue_size_(size)
        {
            Init();
        }
        void Init()
        {
            pub_ = nh_->advertise<T>(topic_name_, queue_size_);
        }

        void Publish(T msg)
        {
            pub_.publish(msg);
        }

        ~RosPublisher() {}

    private:
        ros::NodeHandle *nh_;
        ros::Publisher pub_;
        std::string topic_name_;
        int queue_size_;
    };
   

} // namespace navigation

#endif // NAVI_ROS_API_H