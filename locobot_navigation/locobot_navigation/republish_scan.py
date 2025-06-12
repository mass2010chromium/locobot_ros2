#!/usr/bin/env python3 -O
"""
Cut out the parts of the laserscan which don't have useful information.
Replace them with NAN.

Also, normalize the laserscanner to always output as if Z is pointed upwards.
The X forward direction is used as the reference zero."""

import time

import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from sensor_msgs.msg import LaserScan

import numpy as np

def flip_laserscan(angle_min, angle_max, scan_arr, intensities):
    return -angle_max, -angle_min, scan_arr[::-1], intensities[::-1]

# TODO: Cut out points blocked by the arm too.
# TODO 2: Can we fill in the dead points? It messes up PG.
def main():
    rclpy.init()

    node = Node('lidar_scan_republisher')

    laser_frame = node.declare_parameter('/laser_frame', 'laser').value
    fix_laser_frame = node.declare_parameter('/fix_laser_frame', 'laser_upright').value
    robot_frame = node.declare_parameter('/robot_frame', 'base_link').value
    data_filename = node.declare_parameter('lidar_suppression_mask', 'unknown_file').value

    buffer = Buffer()
    listener = TransformListener(buffer, node)

    while True:
        try:
            transform = buffer.lookup_transform(robot_frame, laser_frame, rclpy.time.Time())

            node.get_logger().info("Got lidar transform")
            break
        except Exception as e:
            node.get_logger().warn("Could not find transform")
            node.get_logger().warn(str(e))

        t0 = time.time()
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            t1 = time.time()
            if t1 - t0 > 1:
                break

    v = Vector3Stamped()
    v.vector.z = 1.0
    res = tf2_geometry_msgs.do_transform_vector3(v, transform)

    eps = 1e-5
    upside_down = abs(res.vector.z - (-1)) < eps
    if upside_down:
        node.get_logger().info("Upside down lidar detected")

    raw_bad_mask = np.load(data_filename)
    extend = 20
    bad_mask = np.logical_or(raw_bad_mask, np.roll(raw_bad_mask, extend))
    bad_mask = np.logical_or(bad_mask, np.roll(raw_bad_mask, -extend))

    node.pub = node.create_publisher(LaserScan, "/scan", 10)
    
    def scan_callback(msg):
        
        scan_arr = np.array(msg.ranges)
        scan_arr[np.isnan(scan_arr)] = np.inf
        scan_arr[bad_mask] = np.nan

        if upside_down:
            (angle_min, angle_max, scan_arr, intensities) = flip_laserscan(msg.angle_min, msg.angle_max, scan_arr, msg.intensities)
        else:
            (angle_min, angle_max, scan_arr, intensities) = (msg.angle_min, msg.angle_max, scan_arr, msg.intensities)

        out_msg = LaserScan()
        #out_msg.header.seq = msg.header.seq
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = fix_laser_frame   # MODIFIED
        out_msg.angle_min = angle_min       # MODIFIED
        out_msg.angle_max = angle_max       # MODIFIED
        out_msg.angle_increment = msg.angle_increment
        out_msg.time_increment = msg.time_increment
        out_msg.scan_time = msg.scan_time
        out_msg.range_min = msg.range_min
        out_msg.range_max = 999.0#msg.range_max
        out_msg.ranges = scan_arr.tolist()           # MODIFIED
        out_msg.intensities = intensities   # MODIFIED
        node.pub.publish(out_msg)
        node.get_logger().info("Pub Scan")

    node.sub = node.create_subscription(LaserScan, "/scan_raw", scan_callback, 10)

    def test(msg):
        node.get_logger().info(f"RECV {msg.range_max}")
    node.sub2 = node.create_subscription(LaserScan, "/scan", test, 10)
    rclpy.spin(node)

if __name__ == "__main__":
    main()

