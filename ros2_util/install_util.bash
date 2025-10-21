#!/usr/bin/env bash
if [ -n "$BASH_VERSION" ]; then
  echo "Symlinking ros2 utilities to current folder..."
else
  echo "Script must be run with bash"
  exit 1
fi

SCRIPT_DIR=$(cd -- "$(dirname -- "$BASH_SOURCE[0]" )" && pwd)

rm -f fastrtps_profile.xml cyclonedds_profile.xml set_env.sh teleop.py restart_ros2.sh
#ln -s "$SCRIPT_DIR/fastrtps_profile.xml" fastrtps_profile.xml
ln -s "$SCRIPT_DIR/cyclonedds_profile.xml" cyclonedds_profile.xml
ln -s "$SCRIPT_DIR/set_env.sh" set_env.sh
ln -s "$SCRIPT_DIR/teleop.py" teleop.py
ln -s "$SCRIPT_DIR/tilt_pan.py" tilt_pan.py
ln -s "$SCRIPT_DIR/restart_ros2.sh" restart_ros2.sh
