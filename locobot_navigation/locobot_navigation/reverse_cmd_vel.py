#!/usr/bin/env python3
"""Simple node to reverse cmd_vel topic for backwards lidar."""

import rospy
from geometry_msgs.msg import Twist

import numpy as np

def main():
    rospy.init_node('velocity_reverser')

    pub = rospy.Publisher("/out_cmd_vel", Twist, queue_size=10)
    
    def twist_callback(msg):
        out_msg = Twist()
        out_msg.angular.x = msg.angular.x
        out_msg.angular.y = msg.angular.y
        out_msg.angular.z = msg.angular.z
        out_msg.linear.x = -msg.linear.x
        out_msg.linear.y = -msg.linear.y
        out_msg.linear.z = -msg.linear.z
        pub.publish(out_msg)

    sub = rospy.Subscriber("/in_cmd_vel", Twist, twist_callback)
    rospy.spin()

if __name__ == "__main__":
    main()

