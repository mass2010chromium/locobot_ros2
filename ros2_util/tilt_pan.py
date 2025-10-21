
import sys, select, termios, tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

msg = """
Tilt and pan the camera!
Pan with with    [ ] 
Tilt with        t g
Adjust increment + -
Zero             0
"""

class TiltPanController(Node):
    def __init__(self):
        super().__init__("tiltpan")
        self.tilt_pub = self.create_publisher(Float64, 'tilt/command', 10)
        self.pan_pub = self.create_publisher(Float64, 'pan/command', 10)
        self.tilt_sub = self.create_subscription(Float64, 'tilt/state', self.tilt_cb, 10)
        self.pan_sub = self.create_subscription(Float64, 'pan/state', self.pan_cb, 10)
        self.tilt = None
        self.pan = None

    def tilt_cb(self, msg):
        self.tilt = msg.data
    def pan_cb(self, msg):
        self.pan = msg.data
    def set_tilt(self, num):
        msg = Float64()
        msg.data = num
        self.tilt_pub.publish(msg)
    def set_pan(self, num):
        msg = Float64()
        msg.data = num
        self.pan_pub.publish(msg)

increment_unit = 0.01
increment_count = 1

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    def getKey():
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    rclpy.init()
    teleop_node = TiltPanController()
    print(msg)
    try:
        while True:
            key = getKey()
            increment = increment_unit * increment_count
            if key == '-':
                increment_count = max(1, increment_count - 1)
                print("Increment =", increment_unit * increment_count)
            elif key == '+':
                increment_count += 1
                print("Increment =", increment_unit * increment_count)
            elif key == 'g':
                target_tilt = teleop_node.tilt + increment_unit * increment_count
                teleop_node.set_tilt(target_tilt)
                print("tilt =", target_tilt)
            elif key == 't':
                target_tilt = teleop_node.tilt - increment_unit * increment_count
                teleop_node.set_tilt(target_tilt)
                print("tilt =", target_tilt)
            elif key == '[':
                target_pan = teleop_node.pan + increment_unit * increment_count
                teleop_node.set_pan(target_pan)
                print("pan =", target_pan)
            elif key == ']':
                target_pan = teleop_node.pan - increment_unit * increment_count
                teleop_node.set_pan(target_pan)
                print("pan =", target_pan)
            elif key == '0':
                teleop_node.set_tilt(0.0)
                teleop_node.set_pan(0.0)
                print("tilt = 0, pan = 0")
            elif key == '\x03':
                break
            for i in range(10):
                rclpy.spin_once(teleop_node, timeout_sec=0.001)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
