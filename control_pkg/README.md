# control_pkg
> Latest update  --  2023.04.23
> 
> 系统：Ubuntu 18.04
>

1、串口权限相关设置——建议永久，详见前一级的基本配置
```
ls -s /dev/ttyUSB
sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1
#永久权限设置
sudo gpasswd -a agv dialout #其中agv是用户名
```

2、control_pkg ————控制功能包
- 参数修改 servo_motor.yaml————串口参数和机器人尺寸参数设置，其中，.yaml中需要修改port_name为当前串口号，例如：/dev/ttyUSB1
- 控制运行，里面订阅/cmd_vel 话题消息计算两侧电机转速
```
roslaunch motor_control servo_motor_control.launch
```
- 手机APP控制，需要在同一个网络环境下（同一个WIFI）
```
rosrun motor_control phoneCommand.py 
```
- 查看并修改ip：
```
ifconfig
```

3、bunker_pkg ————松灵bunker的控制功能包
运行命令
```
#控制连接
sudo -S ip link set can0 up type can bitrate 500000
roslaunch bunker_bringup bunker_robot_base.launch
#键盘控制（需要在启动上述控制连接后）
roslaunch bunker_bringup bunker_teleop_keyboard.launch 
```