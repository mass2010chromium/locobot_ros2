from functools import cached_property

import os
import numpy as np

class EgoCircle:
    def __init__(self, max_range, num_points, far_range_scale=100):
        self.num_points = num_points
        self.angles = np.linspace(-np.pi, np.pi, self.num_points, endpoint=False)
        self.angle_increment = self.angles[1] - self.angles[0]
        self.angle_min = self.angles[0]
        self.angle_max = self.angles[-1]
        self.max_range = max_range
        self.far_range = far_range_scale * self.max_range

        # entries are points [x, y, 1]
        self.points = []
        self.point_priority = []
        self.point_stamps = []
        self.latest_stamp = 0

    def add_scan_points(self, points, priority=0, stamp=None):
        """
        Add a scan measurement from a 3d point cloud.

        @param points Nx3 array of points (xyz) in world frame
        @param priority point priority (default 0)
        @param stamp recency (default incrementing)
        """

        # Homogeneous coordinates.
        if stamp is None:
            # Arbitrary increment to be larger.
            stamp = round(self.latest_stamp) + 1
        self.latest_stamp = stamp

        # Transform into global frame, and record.
        self.points.extend(points)
        self.point_priority.extend([priority] * len(points))
        self.point_stamps.extend([stamp] * len(points))

        
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
        ## Replace INF with far_range (for projection)
        #scan[scan > self.far_range] = self.far_range

        valid_mask = scan < self.max_range
        scan = scan[valid_mask]
        scan_angles = scan_angles[valid_mask]

        # Homogeneous coordinates.
        points = np.array([
                            np.cos(scan_angles),
                            np.sin(scan_angles),
                            np.zeros(len(scan))
                          ]) * scan
        points[2] = 1

        # Transform into global frame, and record.
        self.add_scan_points((pose_2d @ points).T, priority, stamp)


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
        angle_index = ((angles + np.pi) / (2*np.pi) * self.num_points).astype(int)

        scan_res = np.zeros(self.num_points) + self.far_range
        scan_priority = [0] * self.num_points
        scan_stamp = [-1] * self.num_points
        scan_points = [None] * self.num_points
        for r, i, priority, stamp, point in zip(
                rs, angle_index, all_priority, all_stamps, all_points_global.T):
            cur_priority = scan_priority[i]
            cur_stamp = scan_stamp[i]
            if priority > cur_priority:
                scan_priority[i] = priority
                scan_res[i] = r
                scan_points[i] = point
            elif priority == cur_priority:
                if stamp > cur_stamp:
                    scan_priority[i] = priority
                    scan_res[i] = r
                    scan_points[i] = point
                if stamp == cur_stamp and r < scan_res[i]:
                    scan_priority[i] = priority
                    scan_res[i] = r
                    scan_points[i] = point
        scan_res[scan_res > self.max_range] = np.inf
        #scan_res = np.minimum(scan_res, self.max_range)

        points_filt = []
        priority_filt = []
        stamp_filt = []

        for point, stamp, priority in zip(scan_points, scan_stamp, scan_priority):
            if point is None:
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

    accumulator = EgoCircle(10, 512)

    rclpy.init()
    node = Node("egocircle")

    laser_frame = "laser_upright"
    base_frame = "base_link"
    map_frame = "map"
    camera_frame = "camera_frame"
    transforms = {
        "laser_pose": (map_frame, laser_frame),
        "pose": (map_frame, base_frame),
        "camera_transform": (map_frame, camera_frame)
    }

    from motionlib import so3, se3
    from open3d.geometry import AxisAlignedBoundingBox
    from datasource import RosTFDatasource, RosRealsenseDatasource, CombinedDatasource
    from datasource.data_utils import form_point_cloud
    tf_source = RosTFDatasource(transforms, spin_thread=False, node=node)
    rs_source = RosRealsenseDatasource("/camera/camera", node=node)
    rs_source.start()
    
    all_source = CombinedDatasource(
        [ rs_source, tf_source ],
        []
    )

    pub = node.create_publisher(LaserScan, '/fused_scan', 10)
    crop_min = np.array([-np.inf, -np.inf, 0.1])
    crop_max = np.array([np.inf, np.inf, 1.0])
    crop_box = AxisAlignedBoundingBox(crop_min, crop_max)

    # from datasource.depth_anything import DepthPredictor
    # import torch
    # torch.set_grad_enabled(False)
    # CUDA_DEVICE = "cuda:0"
    # device = torch.device(CUDA_DEVICE if torch.cuda.is_available() else "cpu")
    # encoder = 'vits'
    # input_size = 518    # Input resolution?
    # checkpoint_path = os.path.join(os.environ['DEPTH_ANYTHING_ROOT'], 'checkpoints', f'depth_anything_v2_{encoder}.pth')
    # depth_predictor = DepthPredictor(checkpoint_path, device, encoder, input_size)

    def scan_callback(msg):
        frame = msg.header.frame_id
        scan_arr = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(scan_arr))
        try:
            frames = all_source.next()
        except:
            print("Could not get data, passing laserscan")
            pub.publish(msg)
            return
            
        scan_pose = np.array(se3.homogeneous(frames['laser_pose']))
        robot_pose = np.array(se3.homogeneous(frames['pose']))
        cam_pose = np.array(se3.homogeneous(frames['camera_transform']))
        accumulator.add_scan(scan_pose, scan_arr, angles, priority=0)
        o3d_pc = form_point_cloud(color_image=frames['color'], depth_image=rs_source.depth_to_meters(frames['depth']),
                                  intrinsics_mat=rs_source.intrinsics, pose=cam_pose).crop(crop_box)
        points = np.asanyarray(o3d_pc.points)
        #accumulator.add_scan_points(points, priority=1)

        scan_sim, points = accumulator.simulate_scan(robot_pose, prune=True)

        out_msg = LaserScan()
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = base_frame
        out_msg.angle_min = accumulator.angle_min
        out_msg.angle_max = accumulator.angle_max
        out_msg.angle_increment = accumulator.angle_increment
        out_msg.range_min = 0.0
        out_msg.range_max = float(accumulator.max_range)
        out_msg.ranges = scan_sim.tolist()
        out_msg.intensities = [1.0]*len(scan_sim)
        pub.publish(out_msg)

    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    print("Spinning forever...")
    rclpy.spin(node)
