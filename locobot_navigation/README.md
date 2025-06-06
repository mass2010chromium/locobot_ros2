# Locobot navigation bunch

Basically runs the lidar.

## Calibration

In one terminal, run `ros2 launch locobot_navigation/lidar_launch.yaml`.

In another terminal, run `scripts/find_lidar_dead_points.py`.

Jostle the robot (move it around manually); don't stick your hands inside the frame.
Wait for a bit, then ctrl-c `find_lidar_dead_points.py`.
It should print out a nice graph highlighting points that didn't change much in the lidar reading,
this will correspond to the points that are "dead" and blocked by the frame pillars.
