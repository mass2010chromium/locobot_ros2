#!/bin/bash


git config --global user.name locobot
git config --global user.email locobot
echo "export EDITOR=vim" >> ~/.bashrc


mkdir -p ~/locobot_drivers/src


cd ~/locobot_drivers
python3 -m venv test_env


cd ~/locobot_drivers/src
git clone https://github.com/Slamtec/sllidar_ros2.git
git clone https://github.com/kobuki-base/kobuki_ros.git
# Explore-lite (ros2 port)
git clone https://github.com/robo-friends/m-explore-ros2.git
# cv_bridge for ros 2 (must be built from source due to numpy dependency)
git clone --branch humble https://github.com/ros-perception/vision_opencv.git
# ROS domain bridge
git clone --branch humble https://github.com/ros2/domain_bridge.git

# TODO: Move these to lab repo
git clone https://github.com/mass2010chromium/locobot_ros2.git
#git clone https://github.com/mass2010chromium/ego_map.git   # This one requires password oops


cd ~/locobot_drivers
bash src/locobot_ros2/ros2_util/install_util.bash
