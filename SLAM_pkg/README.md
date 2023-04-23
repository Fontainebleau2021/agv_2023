# SLAM_pkg
> Latest update  --  2023.04.23
> 
> 系统：Ubuntu 18.04
>
> 最新版更新了车辆模型的载入

1、lio_sam_devel ————liosam的SLAM定位建图算法，详见功能包中的readme
- 参数配置详见SLAM_pkg/lio_sam_devel/config/params.yaml文件(不同传感器的liosam算法需要修改对应后缀的yaml文件)
- 运行
```
roslaunch lio_sam run.launch
#启动相应传感器的liosam算法，例如启动livox的liosam算法
roslaunch lio_sam run_livox.launch
```
- 车辆载入
```
rosrun lio_sam car_pub.py
```