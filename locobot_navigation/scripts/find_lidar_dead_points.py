import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import numpy as np
import matplotlib.pyplot as plt

def collect():
    CLOSE_THRESHOLD = 0.5

    all_data = []
    def callback(msg):
        scan_arr = np.array(msg.ranges)
        print(scan_arr.shape)
        scan_arr[np.isnan(scan_arr)] = msg.range_max
        all_data.append(scan_arr)

    rclpy.init()
    node = Node('find_dead_angles')
    sub = node.create_subscription(LaserScan, '/scan_raw', callback, 10)
    try:
        rclpy.spin(node)
    except:
        print("Done!")

    print("Got", len(all_data), "scans!")
    all_data = np.array(all_data)
    np.save("scan_data.npy", all_data)
    variances = np.var(all_data, axis=0)
    close_mask = np.all(all_data < CLOSE_THRESHOLD, axis=0)
    plt.figure()
    plt.plot(variances)
    plt.plot(close_mask)
    np.save("scan_variances.npy", variances)
    np.save("scan_close_heuristic.npy", close_mask)
    plt.show()

if __name__ == "__main__":
    collect()

