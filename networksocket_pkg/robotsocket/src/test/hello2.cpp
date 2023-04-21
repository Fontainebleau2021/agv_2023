#include"robotsocket/hello.h"
using namespace std;

void helloworld(){
    ROS_INFO("helloworld");
}

int main(int argc, char *argv[])
{
    HelloPub helloPub;
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"test_head_node");
    helloPub.run();
    helloworld();
    helloPub.test1();
    return 0;
}