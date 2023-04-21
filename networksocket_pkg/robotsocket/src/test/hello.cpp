#include "robotsocket/hello.h"

using namespace std; 

void test(){
    printf("test");
}

void HelloPub::run(){
    ROS_INFO("自定义头文件的使用....");
    test();
}

