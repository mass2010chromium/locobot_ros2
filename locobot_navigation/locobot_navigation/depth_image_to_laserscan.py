#!/usr/bin/env python
# TODO: Write this all in c++
import os
os.environ['MKL_NUM_THREADS'] = "1"
os.environ['OPENBLAS_NUM_THREADS'] = "1"

import rclpy
import tf
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan

import time

import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs

def from_quaternion_xyz(xyzq):
    """
    Convert a pose 7-vector (x, y, z, qx, qy, qz, qw) of translation
    and quaternion into a so3-style transform pair (R, t).
    """
    x,y,z,w = xyzq[3:7]
    x2 = x + x; y2 = y + y; z2 = z + z;
    xx = x * x2;   xy = x * y2;   xz = x * z2;
    yy = y * y2;   yz = y * z2;   zz = z * z2;
    wx = w * x2;   wy = w * y2;   wz = w * z2;

    a11 = 1.0 - (yy + zz)
    a12 = xy - wz
    a13 = xz + wy
    a21 = xy + wz
    a22 = 1.0 - (xx + zz)
    a23 = yz - wx
    a31 = xz - wy
    a32 = yz + wx
    a33 = 1.0 - (xx + yy)
    return np.array([[a11,a12,a13],
                     [a21,a22,a23],
                     [a31,a32,a33]]), np.array(xyzq[:3])

if __name__ == "__main__":

    pipe = rs.pipeline()
    align_to = rs.stream.color
    # These are transforms:
    align = rs.align(align_to)
    pc = rs.pointcloud()

    config = rs.config()
    #config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipe.start(config)

    rclpy.init_node('depth_image_to_laserscan')
    pub = rclpy.Publisher("scan", LaserScan)
    tf_listener = tf.TransformListener()

    camera_tf = np.eye
    base_frame = 'base_link'
    cam_frame = 'camera_frame'
    
    camera_transform = (np.eye(3), np.zeros(3))
    z_min = 0.05
    z_max = 0.7

    num_scan_points = 512
    angle_delta = 2*np.pi / num_scan_points
    range_max = 3
    range_min = 0.1
    time_increment = 0
    scan_time = 0.05
    angle_min = -np.pi
    angle_max = np.pi-angle_delta
    intensities=[]

    t0 = time.time()
    try:
        while not rclpy.is_shutdown():
            if tf_listener.frameExists(base_frame) and tf_listener.frameExists(cam_frame):
                try:
                    t = tf_listener.getLatestCommonTime(base_frame, cam_frame)
                    pos, quat = tf_listener.lookupTransform(base_frame, cam_frame, t)
                    camera_transform = from_quaternion_xyz([*pos, *quat])
                except:
                    print("Could not get transform...")

            frames = pipe.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            points = np.asanyarray(pc.calculate(depth_frame).get_vertices())
            points_np = points.view(np.float32).reshape((*points.shape, -1))
            trans_pts = (camera_transform[0] @ points_np.T).T + camera_transform[1]
            trans_pts = trans_pts[trans_pts[:, 2] < z_max]
            trans_pts = trans_pts[trans_pts[:, 2] > z_min]

            if len(trans_pts) == 0:
                continue

            # TODO: wraparound
            angles = np.arctan2(trans_pts[:, 1], trans_pts[:, 0])
            sort_ind = np.argsort(angles)
            rays_sorted = trans_pts[sort_ind]
            angles_sorted = angles[sort_ind]

            ranges = np.repeat(0, num_scan_points)
            angle_sweep = np.arange(-(num_scan_points//2), (num_scan_points//2), dtype=float) * angle_delta
            orth_vectors = np.vstack([np.cos(angle_sweep), np.sin(angle_sweep), np.zeros(len(angle_sweep))]).T
            pt_start_idx = 0
            for i in range(num_scan_points):
                pt_end_idx = np.searchsorted(angles_sorted, angle_sweep[i], side='right')
                if pt_end_idx != pt_start_idx:
                    # Jumped forward! Extract ray distances that were between these angles.
                    ranges[i] = np.min(np.dot(rays_sorted[pt_start_idx:pt_end_idx], orth_vectors[i]))
                    if ranges[i] < range_min:
                        ranges[i] = 0
                    pt_start_idx = pt_end_idx

            out_msg = LaserScan()
            out_msg.header = Header(stamp=rclpy.Time.now(), frame_id=base_frame)
            out_msg.angle_min = angle_min
            out_msg.angle_max = angle_max
            out_msg.angle_increment = angle_delta
            out_msg.time_increment = time_increment
            out_msg.scan_time = scan_time
            out_msg.range_min = range_min
            out_msg.range_max = range_max
            out_msg.ranges = ranges
            out_msg.intensities = intensities
            pub.publish(out_msg)

            t1 = time.time()
            print(t1 - t0)
            t0 = t1


    finally:
        pipe.stop()
