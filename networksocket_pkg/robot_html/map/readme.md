0、配置环境
sudo apt-get install ros-melodic-turtlebot3-*

gedit .bashrc
最后一行加上export TURTLEBOT3_MODEL=burger

1、启动turtlebot
roslaunch turtlebot3_gazebo turtlebot3_world.launch

2、启动建图、导航rviz
2.1 turtlebot
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_web/map/config/map.yaml
要把导航的起始位置重新定一下

2.2崇明地图
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_web/map/config/nav.yaml
崇明地图可能不用改

3、机器人位置发布
rosrun robot_pose_publisher robot_pose_publisher

4、rosbridge
roslaunch rosbridge_server rosbridge_websocket.launch

5、组网
pgyvpn
