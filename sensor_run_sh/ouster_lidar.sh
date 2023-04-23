#! /bin/bash
roslaunch ouster_ros sensor.launch sensor_hostname:=os-122149001448.local lidar_mode:=1024x10 viz:=true metadata:=$PWD/metadata.json

