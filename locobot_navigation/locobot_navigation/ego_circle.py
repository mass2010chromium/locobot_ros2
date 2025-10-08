from functools import cached_property

import numpy as np

from datasource import RosTFDatasource

class EgoCircle:
    def __init__(self, max_range, num_points, far_range_scale=100):
        self.num_points = num_points
        self.angles = np.linspace(-np.pi, np.pi, self.num_points, endpoint=False)
        self.angle_min = self.angles[0]
        self.angle_max = self.angles[-1]
        self.max_range = max_range
        self.far_range = far_range_scale * self.max_range

        # entries are points [x, y, 1]
        self.points = []
        self.point_priority = []
        self.point_stamps = []
        self.latest_stamp = 0

    def add_scan(self, pose, scan, scan_angles=None, priority=0, stamp=None):
        """
        Add a scan measurement to this egocircle.

        Parameters:
        -----------------
        pose:           4x4 homogeneous transform
        scan:           Laserscan (length N array of ranges)
        scan_angles:    Angles for scan values
                            default: assume scan ranges from [-pi, pi)
        priority:       Clearing priority. Higher priority scans
                            will overwrite lower priority ones.
        stamp:          Scan timestamp (float or None). Used for clearing

        Return:
            None.
        """
        # TODO: store local pose?
        # Points can grow unbounded, leading to loss in precision.
        # Worry about scaling later.
        pose_2d = np.zeros((3, 3))
        pose_2d[:2, :2] = pose[:2, :2]
        pose_2d[:2, 2] = pose[:2, 3]
        pose_2d[2, 2] = 1

        if stamp is None:
            # Arbitrary increment to be larger.
            stamp = round(self.latest_stamp) + 1
        self.latest_stamp = stamp

        if scan_angles is None:
            scan_angles = np.linspace(-np.pi, np.pi, len(scan))

        # Remove NaNs.
        keep_indices = np.logical_not(np.isnan(scan))
        scan = scan[keep_indices]
        scan_angles = scan_angles[keep_indices]
        # Replace INF with far_range (for projection)
        scan[scan > self.far_range] = self.far_range

        # Homogeneous coordinates.
        points = np.array([
                            np.cos(scan_angles),
                            np.sin(scan_angles),
                            np.zeros(len(scan))
                          ]) * scan
        points[2] = 1

        # Transform into global frame, and record.
        self.points.extend((pose_2d @ points).T)
        self.point_priority.extend([priority] * len(scan))
        self.point_stamps.extend([stamp] * len(scan))

    def simulate_scan(self, pose, prune=True):
        """
        Generate a laserscan at the specified robot pose.
        Only looks in 2D.
        Optionally(default) prune the scan points based on priority.

        Return: (scan_result, local_pcd)
        """
        pose_2d = np.zeros((3, 3))
        pose_2d[:2, :2] = pose[:2, :2]
        pose_2d[:2, 2] = pose[:2, 3]
        pose_2d[2, 2] = 1

        all_points_global = np.array(self.points).T
        all_priority = self.point_priority
        all_stamps = self.point_stamps
        pose_inv = np.linalg.inv(pose_2d)
        local_points = pose_inv @ all_points_global

        rs = np.linalg.norm(local_points[:2, :], axis=0)
        angles = np.arctan2(local_points[1, :], local_points[0, :])
        angle_index = int((angles + np.pi) / (2*np.pi) * self.num_points)

        scan_res = np.zeros(self.num_points) + self.far_range
        scan_priority = [0] * len(self.num_points)
        scan_stamp = [-1] * len(self.num_points)
        scan_points = [None] * len(self.num_points)
        for r, i, priority, stamp in zip(
                rs, angle_index, all_priority, all_stamps):
            cur_priority = scan_priority[angle_index]
            cur_stamp = scan_stamp[angle_index]
            if priority >= cur_priority:
                if priority > cur_priority or stamp > cur_stamp:
                    scan_priority[angle_index] = priority
                    scan_res[angle_index] = r
                    scan_points[angle_index] = all_points_global[i]
        scan_res = np.minimum(scan_res, self.max_range)

        points_filt = []
        priority_filt = []
        stamp_filt = []

        for point, stamp, priority in zip(scan_points, scan_stamp, scan_priority):
            if points_filt is None:
                continue
            points_filt.append(point)
            priority_filt.append(priority)
            stamp_filt.append(stamp)
        if prune:
            self.points = points_filt
            self.point_priority = priority_filt
            self.point_stamps = stamp_filt

        return scan_res, pose_inv @ np.array(points_filt).T



if __name__ == '__main__':
    import rclpy
    from rclpy.node import Node

    from sensor_msgs.msg import LaserScan

    accumulator = EgoCircle(3, 512)

    rclpy.init()
    node = Node("egocircle")
    transforms = {
        "pose": ("map", "base_scan")
    }
    tf_source = RosTFDatasource(transforms, spin_thread=False, node=node)

    pub = node.create_publisher(LaserScan, '/scan', 10)

    def scan_callback(msg):
        frame = msg.header.frame_id
        scan_arr = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(scan_arr))
        accumulator.add_scan(np.eye(4), scan_arr, angles)
        print(tf_source.next())

    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    rclpy.spin(node)
