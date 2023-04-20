Navigation Package
=============================
1-costmap 文件夹
（1）costmap.h  costmap_2d.cpp  定义二维栅格地图类实现
（2）costmap_ros.h costmap_ros.cpp 定义接入ROS的二维栅格地图
（3—待完成）costmap_convert.h costmap_convert.cpp 用来将栅格地图转换成多边形表示的障碍物—未完成。后面可以用在costmap_ros里面

2-GLobal_planne文件夹
（1）global_planner.h  定义了全局规划器的基类
（2）a_star.h  a_star.cpp A*的规划算法类实现
（3）其余的是一些其他的规划算法类实现 

3-teb_local_planner 
开源的teb_local_planner类 

4-navi_master 导航文件夹
（1）navi_state.h navi_state.cpp  导航状态机的实现。定义了基类 和各类导航状态下需要完成的工作内容
（2）navigation_ros.h  navigation_ros.cpp   导航实际功能的实现 定义NaviMasterRos类
其中包含 GLobal_planner teb_local_planner 等规划器的对象进行规划， costmap_ros对象接入地图
其中LocalMapObsUpdate函数是为了teb_local_planner需要的多边形障碍物信息类写的 因为1-（3）未完成
（3）seed_line_pose.h .cpp  计算每一跨每一垄播种路径直线的起点和终点位置坐标——（地图的原点如果更新了这个函数需要更新）

5-PID_control 文件夹
（1）pid.h  pid.cpp  实现了一个pid类，能够根据目标值和当前值计算输出
（2）pid_ros.h 基于（1）实现了能读取ros中参数的pidros类，便于在控制中从ros读取参数
（3）seed_control.h 播种机需要用到的pid控制 播种走直线 以及原地转弯等需要使用的控制 根据位置计算机器人的线速度和角速度
（还有另外一种实现是分别计算左右轮的速度）

（4—测试用）PID_control.h PID_control.cpp 这个是单独运行用来测试PID控制的代码


6-navigation包的使用
1：进行机器人的导航 roslaunch navigation navigation_node.launch——可以发布目标点机器人自主运动到目标点
2：其他小功能包类的测试可以自己写main.cpp文件运行，

7—TODO LIST：
（1）navigation_ros.h 文件中NaviMasterRos 需要增加和远程控制界面相关的话题订阅 根据订阅消息更新navi_cmd。更新作业目的地 计算第几跨第几垄
（2）navigation_ros.h LineControl（）中需要根据作业方向 设置内部直线的倾角heading

void SeedLineControlState::Excute(NaviMasterRos *navi_master) 需要更新
void SeedTurnControlState::Excute(NaviMasterRos *navi_master) 需要更新
void SeedUpDownControlState::Excute(NaviMasterRos *navi_master) 需要更新


