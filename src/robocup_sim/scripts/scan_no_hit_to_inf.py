#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Prepublikuje LaserScan a luce bez odrazu (Isaac Sim RTX lidar ich hlasi ako
0.0) prepise na +inf, ako to robi Hokuyo aj Gazebo ray senzor. slam_toolbox
by inak bral nuly ako body v strede robota a robil falosne loop closures.

Pouzitie (v robocup.launch.py):  /scan_raw -> /scan
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanNoHitToInf(Node):
    def __init__(self):
        super().__init__('scan_no_hit_to_inf')
        # Reliable matches both best-effort (costmaps) and reliable subscribers.
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
