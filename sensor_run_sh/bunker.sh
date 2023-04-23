#! /bin/bash
echo "agv" | sudo -S ip link set can0 up type can bitrate 500000

sleep 2

roslaunch bunker_bringup bunker_robot_base.launch
