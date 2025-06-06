#!/bin/bash

# Starters
apt-get update
apt-get upgrade



# Create locobot user. (with dummy password)
useradd -s /bin/bash locobot
echo "locobot:locobot" | sudo chpasswd



# Quality of life
apt-get install tmux vim net-tools



# Set up SSH, and disable ssh by password.
# This part is interactive as it requires user interaction.
apt-get install openssh-server

read -n 1 -s -r -p "Waiting for user to set up ssh by key..."
echo "" # Move to the next line
# Edit ssh config to disable password ssh
sed -i '/PasswordAuthentication/c\PasswordAuthentication No.' /etc/ssh/sshd_config
service ssh restart



#########################################
# BEGIN ROS 2 main install
#########################################
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Adding ros 2 apt repository
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo apt install /tmp/ros2-apt-source.deb

# Install ROS 2
sudo apt update; sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
#########################################
# END ROS 2 main install
#########################################



# Build tools install
sudo apt install -y python3.10-venv build-essential python3-colcon-common-extensions python3-rosdep ros-humble-rmw-cyclonedds-cpp
