# catkin_web
崇明项目前端模块代码。
需要使用frp组网
https://blog.csdn.net/qq_40695642/article/details/126819203

## 依赖
sudo apt-get install ros-melodic-tf2-sensor-msgs 

## 项目运行
依赖
sudo apt-get install ros-melodic-map-server
sudo apt-get install ros-melodic-web-video-server
sudo apt-get install ros-melodic-rosbridge-suite


### 一、 环境
```
source devel/setup.bash
```

### 二、运行rosbridge
```
roslaunch rosbridge_server rosbridge_websocket.launch 
```

### 三、测试前端功能
---
### 1. 测试摄像头传输  

(1) 运行web_video_server
```
rosrun web_video_server web_video_server
```
(2) 播放rosbag(catkin_camera)
```C++
roslaunch camera camera.launch
```
---
### 2. 测试电子地图

(0)此条命令可以发布地图
rosrun map_server map_server /home/agv/catkin_web/map/config/nav.yaml
rosrun robot_pose_publisher robot_pose_publisher

不需要下面的了
-----------

(1) 启动turtlebot3（需要安装依赖）
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
(2) 启动建图、导航功能  
- (a) turtlebot3测试
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_web/map/config/map.yaml
```
  - (b) 崇明地图测试
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_web/map/config/nav.yaml
```
(3) 机器人位置发布
```
rosrun robot_pose_publisher robot_pose_publisher
```

