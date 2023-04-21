# planning_ws
1：串口权限设置——建议永久 详情百度
ls -s /dev/ttyUSB
sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1

2：servo_motor.yaml 串口参数和机器人尺寸参数设置
3：roslaunch motor_control servo_motor_control.launch
 里面订阅/cmd_vel 话题消息计算两侧电机转速

4.rosrun motor_control phoneCommand.py 


//motor_control_test/
.yaml: port_name: /dev/ttyUSB1

ls -s dev/ttyUSB tab tab
sudo chmod dev/ttyUSB0
sudo chmod dev/ttyUSB1

cd motor_control_test
source devel/setup.bash
roslaunch motor_control servo_motor_control.launch

cd motor_control_test
source devel/setup.bash
rosrun motor_control phoneCommand.py 

ifconfig
