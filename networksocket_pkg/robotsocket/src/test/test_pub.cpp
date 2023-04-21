#include <ros/ros.h>
#include"robotsocket/state.h"

int main(int argc, char **argv)
{
        ros::init(argc, argv,"robot_pub");
        ros::NodeHandle n;
        ros::Publisher publish = n.advertise<robotsocket::state>("/state", 10);
        ros::Rate loop_rate(10);
        int count=0;
        while (ros::ok())
        {
                robotsocket::state msg;
                msg.x = count;
                msg.y = 2;
                msg.state = 1;
                publish.publish(msg);
                printf("x=%f, y=%f, state=%d, count=%d\n",msg.x,msg.y,msg.state,count);
                loop_rate.sleep();
                count++;
        }

        return 0;
}
