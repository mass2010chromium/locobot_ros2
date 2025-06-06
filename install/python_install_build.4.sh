#!/bin/bash


cd ~/locobot_drivers/
cp install -r src/locobot_ros2/install/.tmux.conf ~/
cp install -r src/locobot_ros2/install/.vimrc ~/


source set_env.sh
pip install -r src/locobot_ros2/install/venv.txt


build
