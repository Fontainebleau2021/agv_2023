# navigation_pkg
> Latest update  --  2023.04.22
> 
> 系统：Ubuntu 18.04
>
1、bunker_description ————导航地图规划中松灵bunker模型加载

2、navigation ————导航功能包，功能包内容详见功能包中的readme
- 参数修改：
    - 导航相关修改详见navigation/config/navigation/navigation_params.yaml文件
    - teb规划下速度限制参数详见navigation/config/teb/teb_local_planner_params.yaml文件
- 运行

```
roslaunch navigation navigation_node.launch 
```
