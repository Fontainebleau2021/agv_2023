# map_pkg
> Latest update  --  2023.04.21
> 
> 系统：Ubuntu 18.04
>
1、map_load功能包 ————地图加载
- 参数修改：
    - map文件夹为二维栅格地图加载文件夹，其中png/pgm文件为地图文件，yaml文件为配置文件，其中：
        - image: map.png　　#文件名
        - resolution: 0.050000　　#地图分辨率 单位：米/像素
        - origin: [-49.0286, -107.401, 0.0]   #图像左下角在地图坐标下的坐标
        - negate: 0    #是否应该颠倒 白：自由/黑：的语义(阈值的解释不受影响)
        - occupied_thresh: 0.65   #占用概率大于此阈值的像素被认为已完全占用
        - free_thresh: 0.196   #用率小于此阈值的像素被认为是完全空闲的
    - launch文件中的tf转换参数为不同frame的静态转换
- 运行：
```
roslaunch map_load map_load.launch 
#or 在rviz中显示
roslaunch map_load map_load_rviz.launch 
```

2、publish_pointcloud ————将三维pcd地图压缩为二维栅格地图并输出
- 参数修改：
    - pcd文件目录————demo.launch中的path
    - 压缩的相关参数在octomaptransform.launch中：
        - resolution 地图分辨率
        - sensor_model/max_range 压缩的范围
        - pointcloud_max_z和pointcloud_min_z 压缩的高度限制
- 运行

```
roslaunch publish_pointcloud demo.launch 
```
