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
rm -rf /tmp/install_tmp
mkdir -p /tmp/install_tmp
cd /tmp/install_tmp
wget https://github.com/IntelRealSense/librealsense/releases/download/v2.56.3/librealsense2_jammy_x86_debians_beta.zip
sudo apt-get install -y libglfw3 v4l-utils libgtk-dev
sudo dpkg -i *.deb
sudo apt-get --fix-broken install -y
#########################################



#########################################
# rplidar
cd /home/locobot/locobot_drivers/src/sllidar_ros2
sudo sh scripts/create_udev_rules.sh
echo "Remember to chmod 777 /dev/ttyUSB*"
echo "find the appropriate one for rplidar and dynamixel using usb_util.py"
#########################################

