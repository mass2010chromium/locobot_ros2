#!/usr/bin/env bash
export ROS_WS_ROOT=$(cd -- "$(dirname -- "$BASH_SOURCE[0]")" && pwd)

#export FASTRTPS_DEFAULT_PROFILES_FILE="$ROS_WS_ROOT/fastrtps_profile.xml"
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export CYCLONEDDS_URI="$ROS_WS_ROOT/cyclonedds_profile.xml"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash

# Python virtualenv
source "$ROS_WS_ROOT/locobot_env/bin/activate"
source "$ROS_WS_ROOT/install/setup.bash"
export ROS_DOMAIN_ID=8

alias rosrun="ros2 run"
alias rostopic="ros2 topic"
alias roslaunch="ros2 launch"
alias rosnode="ros2 node"
alias rviz="ros2 run rviz2 rviz2"

build() {
    # NOTE: symlink-install is breaking python imports -- why?
    #(cd $ROS_WS_ROOT; colcon build --symlink-install)
    (cd $ROS_WS_ROOT; colcon build "$@")
    source "$ROS_WS_ROOT/install/setup.bash"
}

clean() {
    (cd $ROS_WS_ROOT; rm -rf build log install)
}
