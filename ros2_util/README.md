# Utility scripts for ROS 2

- `fastrtps_profile.xml`: Example fastrtps unicast configuration

- `restart_ros2.sh`: Restart the ROS2 daemon because apparently that's a common error

- `set_env.sh`: Convenience script I use for setting ros2 workspace.
    Aliases things to be easier to type, set up fastrtps profile, source
    `/opt/ros/.../setup.bash` here so I don't have to put it in bashrc.
    Also source python virtualenv and set `ROS_DOMAIN_ID`, and define
    convenience bash functions for building and cleaning the project.

- `teleop.py`: Keyboard teleop ported to ros 2

- `install_util.sh`: Symlink these scripts/configs into the current directory

## Annoyance scripts

- `ros1_to_ros2.py`: Substitution script to attempt to ease the pain of converting ROS1 c++ code to ROS2
