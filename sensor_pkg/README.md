# sensor_pkg
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

2、相应传感器的功能包，详见上一级配置文件的相关操作命令