# control_pkg
> Latest update  --  2023.04.21
> 
> 系统：Ubuntu 18.04
>
1、串口权限设置——建议永久，详见前一级的基本配置
```
ls -s /dev/ttyUSB
sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1
```

2、servo_motor.yaml————串口参数和机器人尺寸参数设置，其中，.yaml中需要修改port_name为当前串口号，例如：/dev/ttyUSB1

3、控制运行
```
roslaunch motor_control servo_motor_control.launch
```
 里面订阅/cmd_vel 话题消息计算两侧电机转速

4、手机APP控制，需要在同一个网络环境下（同一个WIFI）
```
rosrun motor_control phoneCommand.py 
```
查看并修改ip：
```
ifconfig
```
