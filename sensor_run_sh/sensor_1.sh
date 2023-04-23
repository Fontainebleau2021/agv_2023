#! /bin/bash
echo "test"

chmod +x ./lidar.sh
gnome-terminal -t "lidar" -x bash -c "sh ./lidar.sh;exec bash"

sleep 2

chmod +x ./imu.sh
gnome-terminal -t "imu" -x bash -c "sh ./imu.sh;exec bash"


sleep 2

chmod +x ./time.sh
gnome-terminal -t "time" -x bash -c "sh ./time.sh;exec bash"

sleep 2

chmod +x ./GPS_0.sh
gnome-terminal -t "GPS" -x bash -c "sh ./GPS_1.sh;exec bash"

sleep 2

chmod +x ./bunker.sh
gnome-terminal -t "bunker" -x bash -c "sh ./bunker.sh;exec bash"


sleep 2

chmod +x ./camera.sh
gnome-terminal -t "camera" -x bash -c "sh ./camera.sh;exec bash"
