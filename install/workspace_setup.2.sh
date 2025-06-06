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
# TODO: Move these to lab repo
git clone https://github.com/mass2010chromium/locobot_ros2.git
git clone https://github.com/mass2010chromium/ego_map.git   # This one requires password oops


cd ~/locobot_drivers
bash src/ego_map/ros2_util/install_util.bash
