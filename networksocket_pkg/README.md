# networksocket_pkg
> Latest update  --  2023.04.22
> 
> 系统：Ubuntu 18.04
>

# catkin_socket
崇明项目socket传输代码

## 代码运行
1. 环境
```
source devel/setup.bash
```


2. 运行roscore
```
roscore
```

3. 运行robotsocket程序，和服务器建立连接
```
rosrun robotsocket robotsocket
```

4. 运行bag(GPS经纬度和xy坐标)（catkin_GPS）
```
roslaunch nmea_navsat_driver gpsStart.launch
```

5. 监听/state话题(不监听的话程序会卡住)
```
rostopic echo /state
```

6. 发布停止指令（02作业指令测试）
```
rostopic pub /state robotsocket/state "state: 1
x: 0.0
y: 0.0" 
```