import time
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from locobot_control.srv import LookCommand

from motionlib import so3, se3, vectorops as vo

IDLE = 0
VIEW = 1
PAUSE = 2

class TiltPanController(Node):

    def __init__(self):
        super().__init__("tiltpan")
        self.tilt_pub = self.create_publisher(Float64, 'tilt/command', 10)
        self.pan_pub = self.create_publisher(Float64, 'pan/command', 10)
        self.tilt_sub = self.create_subscription(Float64, 'tilt/state', self.tilt_cb, 10)
        self.pan_sub = self.create_subscription(Float64, 'pan/state', self.pan_cb, 10)
        self.vel_sub = self.create_subscription(Twist, 'cmd_vel', self.vel_cb, 10)

        self.command_service = self.create_service(LookCommand, 'look_cmd', self.look_cb)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.mode = IDLE

        # Idle controller parameters
        self.t = time.monotonic()
        self.sim_t = self.t
        self.sim_step = 0.01
        self.orbit_vec = np.array([0.0, 1.0])
        self.prev_vel = None    # vx, vy numpy array.
        self.idle_tilt = -0.1
        self.idle_pan_limit = 0.9

        # Look-at controller parameters
        self.target = None  # If None, will go to idle mode
        self.map_frame = "map"
        self.camera_pivot_frame = "head_tilt_link"
        self.base_frame = "base_link"
        self.overshoot_dt = 0.25 # Tuned lol

        # Pause controller parameters
        self.target_tilt = None
        self.target_pan = None

        self.tilt = None
        self.pan = None
        self.vel = [0, 0, 0]

    def set_idle_mode(self):
        self.mode = IDLE
        self.t = time.monotonic()
        self.sim_t = self.t
        self.orbit_vec = np.array([self.pan, 1.0])
        self.prev_vel = None    # vx, vy numpy array.

    def set_view_mode(self):
        """Set to look at a location, specified in the map frame."""
        self.mode = VIEW

    def set_pause_mode(self):
        """Set to look at a fixed orientation"""
        self.mode = PAUSE

    def set_target(self, target_pos: "vec3 | None"):
        self.target = target_pos
        if target_pos is None:
            self.set_idle_mode()

    def update(self):
        # Call in mainloop
        if self.mode == IDLE:
            center = [self.vel[2]*1.0, 0.0]
            amplitude = max(0, min(0.8, 0.8 - self.vel[0] * 2))

            self.t = time.monotonic()
            while self.sim_t < self.t:
                self.update_idle_state(center, amplitude, self.sim_step)
                self.sim_t += self.sim_step
            self.set_tilt(self.idle_tilt)
            self.set_pan(np.clip(self.orbit_vec[0], -self.idle_pan_limit, self.idle_pan_limit))
        elif self.mode == VIEW:
            self.track_target()
        elif self.mode == PAUSE:
            self.set_tilt(self.target_tilt)
            self.set_pan(self.target_pan)


    def update_idle_state(self, center, amplitude, dt):
        """
        Compute the desired state velocity.
        """
        r_vec = self.orbit_vec - center
        r_mag = np.linalg.norm(r_vec)
        angle = np.atan2(r_vec[1], r_vec[0])
        if r_mag < 1e-4:
            if self.prev_vel is None:
                return
            self.orbit_vec += self.prev_vel * dt
            return

        dr = amplitude - r_mag
        v_rad = dr * np.array([np.cos(angle), np.sin(angle)])   # TODO: scaling
        # Interpolating between v_tang=2r at origin, and v_tang=r at r=1
        v_tang = (2 - r_mag) * r_mag * np.array([-np.sin(angle), np.cos(angle)])
        v_net = v_rad + v_tang
        self.orbit_vec += v_net * dt
        self.prev_vel = v_net

    def track_target(self):
        """ Turn the head towards a position specified in the map frame. """
        # "Python atomic" thread safety
        target = self.target
        vel = self.vel
        if target is None:
            self.set_idle_mode()
            return

        # Lookup transforms
        try:
            ros_time = rclpy.time.Time()
            T = self.tf_buffer.lookup_transform(self.base_frame, self.map_frame, ros_time).transform
            T2 = self.tf_buffer.lookup_transform(self.base_frame, self.camera_pivot_frame, ros_time).transform
        except TransformException as ex:
            # NOTE: the comment isn't wrong, ros convention is "the pose of the b in a" which is actually "the transform from b to a"
            print(f"Could not get [{self.map_frame}] -> [{self.camera_pivot_frame}] transform...")
            print(ex)
            return

        pos = [T.translation.x, T.translation.y, T.translation.z]
        base_to_head = [T2.translation.x, T2.translation.y, T2.translation.z]
        quat = [T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z]
        map_to_head = (so3.from_quaternion(quat), vo.sub(pos, base_to_head))

        vel_fudge = np.array(vel) * self.overshoot_dt
        vel_fudge = (so3.from_moment((0, 0, vel_fudge[2])), (vel_fudge[0], vel_fudge[1], 0))

        target_local = se3.apply(se3.inv(vel_fudge), se3.apply(map_to_head, target))

        # XY vector
        horiz = np.linalg.norm(target_local[:2])
        pitch = np.atan2(target_local[2], horiz)
        if horiz < 1e-4:
            yaw = 0.0
        else:
            yaw = np.atan2(target_local[1], target_local[0])

        pitch = np.clip(pitch, -0.3, 0.5)
        yaw = np.clip(yaw, -2.5, 2.5)
        self.set_tilt(-pitch)
        self.set_pan(yaw)


    def tilt_cb(self, msg):
        self.tilt = msg.data
    def pan_cb(self, msg):
        self.pan = msg.data
    def vel_cb(self, msg):
        self.vel = [msg.linear.x, msg.linear.y, msg.angular.z]
    def look_cb(self, request, response):
        mode = request.mode
        response.result = True
        if mode == IDLE:
            self.set_idle_mode()
            self.idle_tilt = request.tilt
        elif mode == VIEW:
            self.set_view_mode()
            self.set_target([request.target_x, request.target_y, request.target_z])
        elif mode == PAUSE:
            self.set_pause_mode()
            self.target_tilt = request.tilt
            self.target_pan = request.pan
        else:
            response.result = False
        return response

    def set_tilt(self, num):
        self.tilt_pub.publish(Float64(data=num))
    def set_pan(self, num):
        self.pan_pub.publish(Float64(data=num))

if __name__ == "__main__":

    rclpy.init()
    teleop_node = TiltPanController()
    #teleop_node.set_target([0.0, 0.0, 1.0])
    #teleop_node.set_view_mode()

    angle_limit = 0.9
    dt = 0.2
    t0 = time.time()
    t = 0
    while True:
        teleop_node.update()
        for i in range(20):
            rclpy.spin_once(teleop_node, timeout_sec=dt/20)
