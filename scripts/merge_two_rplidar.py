#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanMerger(Node):
    def __init__(self):
        super().__init__('scan_merger')
        self.sub_front_left = self.create_subscription(
            LaserScan, '/scan_front_left', self.front_left_callback, 10)
        self.sub_back_right = self.create_subscription(
            LaserScan, '/scan_back_right', self.back_right_callback, 10)
        self.publisher_ = self.create_publisher(LaserScan, '/scan_360', 10)
        self.front_left_scan = None
        self.back_right_scan = None

    def front_left_callback(self, msg):
        self.front_left_scan = msg
        self.merge_scans()

    def back_right_callback(self, msg):
        self.back_right_scan = msg
        self.merge_scans()

    def merge_scans(self):
        if self.front_left_scan and self.back_right_scan:
            merged_scan = LaserScan()
            # Copy data from front_left_scan and back_right_scan, ensure proper angle alignment
            # Example: assuming front_left_scan has angles from 0 to 180 and back_right_scan from 180 to 360
            merged_scan.ranges = self.front_left_scan.ranges + self.back_right_scan.ranges
            merged_scan.angle_min = 0.0
            merged_scan.angle_max = 6.28  # 2 * pi (360 degrees)
            merged_scan.angle_increment = self.front_left_scan.angle_increment
            merged_scan.range_min = min(self.front_left_scan.range_min, self.back_right_scan.range_min)
            merged_scan.range_max = max(self.front_left_scan.range_max, self.back_right_scan.range_max)
            self.publisher_.publish(merged_scan)

def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
