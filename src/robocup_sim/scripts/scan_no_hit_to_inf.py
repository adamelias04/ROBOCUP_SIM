#!/usr/bin/env python3
# Preposiela /scan_raw na /scan a luce bez odrazu (RTX lidar v Isaacu ich hlasi
# ako 0.0) prepise na inf, ako realny Hokuyo. slam_toolbox by inak bral nuly ako
# body v strede robota. Spusta ho robocup.launch.py.
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanNoHitToInf(Node):
    def __init__(self):
        super().__init__('scan_no_hit_to_inf')
        self.pub = self.create_publisher(LaserScan, 'scan_out', 10)
        self.create_subscription(LaserScan, 'scan_in', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        msg.ranges = [math.inf if r < msg.range_min else r for r in msg.ranges]
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ScanNoHitToInf())


if __name__ == '__main__':
    main()
