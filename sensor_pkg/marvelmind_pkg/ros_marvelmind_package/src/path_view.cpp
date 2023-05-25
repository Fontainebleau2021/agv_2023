#include "ros/ros.h"
#include "std_msgs/String.h"
#include "marvelmind_nav/hedge_pos.h"
#include <geometry_msgs/PoseStamped.h>;
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>;
#include <tf/transform_datatypes.h>
#include <math.h>;

#define HEDGE_POSITION_TOPIC_NAME "hedge_pos"

struct my_pose
{
    double x_pose;
    double y_pose;
    double z_pose;
};

int pose_num;
double x_error;
double x_max_error;
double y_error;
double y_max_error;
std::vector<double> poses_x;
std::vector<double> poses_y;
ros::Publisher state_pub_;
ros::Publisher pose_pub_;
ros::Publisher odom_pub_;
ros::Publisher odom_zone_pub_;
nav_msgs::Path ros_path_;
bool init;
my_pose init_pose;
my_pose last_pose;

boost::array<double, 36> ODOM_POSE_COVARIANCE = {
  {1e-3, 0, 0, 0, 0, 0,
   0, 1e-3, 0, 0, 0, 0,
   0, 0, 1e6, 0, 0, 0,
   0, 0, 0, 1e6, 0, 0,
   0, 0, 0, 0, 1e6, 0,
   0, 0, 0, 0, 0, 1e-3}};

boost::array<double, 36> ODOM_POSE_COVARIANCE2 = {
  {1e-9, 0, 0, 0, 0, 0, 
   0, 1e-3, 1e-9, 0, 0, 0,
   0, 0, 1e6, 0, 0, 0,
   0, 0, 0, 1e6, 0, 0,
   0, 0, 0, 0, 1e6, 0,
   0, 0, 0, 0, 0, 1e-9}};

boost::array<double, 36> ODOM_TWIST_COVARIANCE = {
  {1e-3, 0, 0, 0, 0, 0,
   0, 1e-3, 0, 0, 0, 0,
   0, 0, 1e6, 0, 0, 0,
   0, 0, 0, 1e6, 0, 0,
   0, 0, 0, 0, 1e6, 0,
   0, 0, 0, 0, 0, 1e-3}};

boost::array<double, 36> ODOM_TWIST_COVARIANCE2 = {
  {1e-9, 0, 0, 0, 0, 0, 
   0, 1e-3, 1e-9, 0, 0, 0,
   0, 0, 1e6, 0, 0, 0,
   0, 0, 0, 1e6, 0, 0,
   0, 0, 0, 0, 1e6, 0,
   0, 0, 0, 0, 0, 1e-9}};


void hedgePos_noaddressCallback(const marvelmind_nav::hedge_pos &hedge_pos_msg)
{
    ROS_INFO("Hedgehog data: Timestamp= %d, X=%.3f  Y= %.3f  Z=%.3f  flags=%d",
             (int)hedge_pos_msg.timestamp_ms,
             (float)hedge_pos_msg.x_m, (float)hedge_pos_msg.y_m, (float)hedge_pos_msg.z_m,
             (int)hedge_pos_msg.flags);

    float x_tolerance = 2.0;
    float y_tolerance = 1.0;

    // if (!init)
    // {
    //     if ((abs(hedge_pos_msg.x_m)        <=x_tolerance)&&(abs(hedge_pos_msg.y_m)        <=y_tolerance)){return;}
    //     if ((abs(hedge_pos_msg.x_m-11.954) <=x_tolerance)&&(abs(hedge_pos_msg.y_m)        <=y_tolerance)){return;}
    //     if ((abs(hedge_pos_msg.x_m-11.732) <=x_tolerance)&&(abs(hedge_pos_msg.y_m-20.229) <=y_tolerance)){return;}
    //     if ((abs(hedge_pos_msg.x_m-0.154)  <=x_tolerance)&&(abs(hedge_pos_msg.y_m-20.185) <=y_tolerance)){return;}
    // }

    if ((hedge_pos_msg.flags & (1 << 0)) == 0)
    {
        if (!init)
        {
            init_pose.x_pose = hedge_pos_msg.x_m;
            init_pose.y_pose = hedge_pos_msg.y_m;
            init_pose.z_pose = hedge_pos_msg.z_m;
            last_pose.x_pose = 0;
            last_pose.y_pose = 0;
            last_pose.z_pose = 0;
            init = true;
            pose_num = 1;
            x_error = 0;
            x_max_error = 0;
            y_error = 0;
            y_max_error = 0;
        }
        else
        {
            double x, y, z;
            geometry_msgs::Quaternion q;
            double k,theta;
            ros_path_.header.frame_id = "marvelmind";
            //ros_path_.header.stamp = ros::Time::now();
            ros::Time hedge_pos_msg_time(hedge_pos_msg.timestamp_ms);
            ros_path_.header.stamp = hedge_pos_msg_time;
            geometry_msgs::PoseStamped pose;
            nav_msgs::Odometry odom;
            pose.header = ros_path_.header;
            odom.header.frame_id = "odom";
            odom.header.stamp = hedge_pos_msg_time;
            x = hedge_pos_msg.x_m - init_pose.x_pose;
            y = hedge_pos_msg.y_m - init_pose.y_pose;
            z = hedge_pos_msg.z_m - init_pose.z_pose;
            pose.pose.position.x = y;
            pose.pose.position.y = -x;
            pose.pose.position.z = 0;
            odom.pose.pose.position.x = pose.pose.position.x;
            odom.pose.pose.position.y = pose.pose.position.y;
            odom.pose.pose.position.z = 0;
            k = (odom.pose.pose.position.y - last_pose.y_pose)/(odom.pose.pose.position.x - last_pose.x_pose);
            theta = atan(k);
            q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, theta);
            odom.pose.pose.orientation.x = q.x;
            odom.pose.pose.orientation.y = q.y;
            odom.pose.pose.orientation.z = q.z;
            odom.pose.pose.orientation.w = q.w;
            odom.pose.covariance = ODOM_POSE_COVARIANCE2;
            odom.twist.covariance = ODOM_TWIST_COVARIANCE2;
            odom.pose.covariance[0] = 1e-3;
            odom.pose.covariance[7] = 1e-3;
            odom.pose.covariance[14] = 1e-3;
            poses_x.push_back(x);
            poses_y.push_back(y);
            if (x >= 0)
            {
                if (x >= x_max_error)
                    x_max_error = x;
            }
            else
            {
                if ((-x) >= x_max_error)
                    x_max_error = -x;
            }
            if (y >= 0)
            {
                if (y >= y_max_error)
                    y_max_error = y;
            }
            else
            {
                if ((-y) >= y_max_error)
                    y_max_error = -y;
            }
            pose_pub_.publish(pose);
            ros_path_.poses.push_back(pose);
            x_error = (x_error * (pose_num - 1) + x) / pose_num;
            y_error = (y_error * (pose_num - 1) + y) / pose_num;
            pose_num = pose_num + 1;
            double v_x = 0;
            double v_y = 0;
            for (int i = 0; i < poses_x.size(); i++)
            {
                v_x = v_x + pow((poses_x[i]-x_error),2);
                v_y = v_y + pow((poses_y[i]-y_error),2);
            }
            v_x = sqrt(v_x/poses_x.size());
            v_y = sqrt(v_y/poses_y.size());
            ROS_INFO("( x:%0.6f ,y:%0.6f ,z:%0.6f)", x, y, z);
            ROS_INFO("----------***x_error:%0.6f", x_error);
            ROS_INFO("----------***x_max_error:%0.6f", x_max_error);
            ROS_INFO("**********---y_error:%0.6f", y_error);
            ROS_INFO("**********---y_max_error:%0.6f", y_max_error);
            ROS_INFO("&&&&&&&&&&***v_x:%0.6f", v_x);
            ROS_INFO("&&&&&&&&&&***v_y:%0.6f", v_y);
            state_pub_.publish(ros_path_);
            odom_zone_pub_.publish(odom);
            last_pose.x_pose = odom.pose.pose.position.x;
            last_pose.y_pose = odom.pose.pose.position.y;
            last_pose.z_pose = z;
            if ((abs(hedge_pos_msg.x_m)        <=x_tolerance)&&(abs(hedge_pos_msg.y_m)        <=y_tolerance)){return;}
            if ((abs(hedge_pos_msg.x_m-11.954) <=x_tolerance)&&(abs(hedge_pos_msg.y_m)        <=y_tolerance)){return;}
            if ((abs(hedge_pos_msg.x_m-11.732) <=x_tolerance)&&(abs(hedge_pos_msg.y_m-20.229) <=y_tolerance)){return;}
            if ((abs(hedge_pos_msg.x_m-0.154)  <=x_tolerance)&&(abs(hedge_pos_msg.y_m-20.185) <=y_tolerance)){return;}
            odom.pose.covariance[0] = 1e-3;
            odom.pose.covariance[7] = 1e-3;
            odom.pose.covariance[14] = 1e-3;
            if ((abs(hedge_pos_msg.x_m)<=x_tolerance)||(abs(hedge_pos_msg.y_m)<=y_tolerance)||(abs(hedge_pos_msg.x_m-11.954)<=x_tolerance)||(abs(hedge_pos_msg.y_m-20.229)<=y_tolerance))
            {
                odom.pose.covariance[0] = 1e-4;
                odom.pose.covariance[7] = 1e-4;
                odom.pose.covariance[14] = 1e-4;
            }
            odom_pub_.publish(odom);
        }
    }
}

int main(int argc, char **argv)
{
    init = false;
    ros::init(argc, argv, "path_view");
    ros::NodeHandle n;
    ros::Subscriber subHedge_noaddress = n.subscribe(HEDGE_POSITION_TOPIC_NAME, 1000, hedgePos_noaddressCallback);

    state_pub_ = n.advertise<nav_msgs::Path>("marvel_path", 10);
    pose_pub_ = n.advertise<geometry_msgs::PoseStamped>("marvel_pose", 10);
    odom_pub_ = n.advertise<nav_msgs::Odometry>("marvel_odom", 200);
    odom_zone_pub_ = n.advertise<nav_msgs::Odometry>("marvel_zone_odom", 200);
    ros::spin();
    return 0;
}
