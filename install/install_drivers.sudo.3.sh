#!/bin/bash

# ROS packages
sudo apt-get install -y \
        ros-humble-topic-tools \
        ros-humble-dynamixel-workbench \
        ros-humble-dynamixel-workbench-msgs \
        ros-humble-control-msgs \
        ros-humble-simple-actions \
        ros-humble-kobuki-core \
        ros-humble-kobuki-ros-interfaces \
        ros-humble-diagnostic-updater \


#########################################
# realsense
mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
apt-get install -y apt-transport-https
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

apt-get install -y librealsense2-dkms \
                   librealsense2-utils \
                   librealsense2-dev \
                   librealsense2-dbg
#########################################



#########################################
# rplidar
cd /home/locobot/locobot_drivers/src/sllidar_ros2
sudo sh scripts/create_udev_rules.sh
echo "Remember to chmod 777 /dev/ttyUSB*"
echo "find the appropriate one for rplidar and dynamixel using usb_util.py"
#########################################

