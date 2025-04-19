#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs

class LaserFilterNode(Node):
    def __init__(self):
        super().__init__('laser_filter_node')

        # สร้าง TF buffer และ listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to the front and back laser scan topics
        self.front_subscription = self.create_subscription(
            LaserScan, 'front/scan', self.front_scan_callback, 10
        )
        self.back_subscription = self.create_subscription(
            LaserScan, 'back/scan', self.back_scan_callback, 10
        )

        # Publisher for the filtered scans
        self.filtered_front_pub = self.create_publisher(LaserScan, '/filtered_front_scan', 10)
        self.filtered_back_pub = self.create_publisher(LaserScan, '/filtered_back_scan', 10)
        self.filtered_combined_pub = self.create_publisher(LaserScan, '/filtered_combined_scan', 10)

        # Store filtered scan data
        self.filtered_front_scan = None
        self.filtered_back_scan = None

    def front_scan_callback(self, msg: LaserScan):
        filtered_msg = self.filter_scan(msg, 120.0, 180.0, -170.0, 0.0)
        self.filtered_front_scan = filtered_msg
        self.filtered_front_pub.publish(filtered_msg)
        self.publish_combined_scan()

    def back_scan_callback(self, msg: LaserScan):
        filtered_msg = self.filter_scan(msg, -63.0, 145.0, 0.0, 0.0)
        self.filtered_back_scan = filtered_msg
        self.filtered_back_pub.publish(filtered_msg)
        self.publish_combined_scan()

    def filter_scan(self, msg: LaserScan, start_angle_deg_1, end_angle_deg_1, start_angle_deg_2, end_angle_deg_2):
        # กรองข้อมูลเลเซอร์สองช่วง
        start_angle_1 = math.radians(start_angle_deg_1)
        end_angle_1 = math.radians(end_angle_deg_1)
        start_angle_2 = math.radians(start_angle_deg_2)
        end_angle_2 = math.radians(end_angle_deg_2)

        start_index_1 = int((start_angle_1 - msg.angle_min) / msg.angle_increment)
        end_index_1 = int((end_angle_1 - msg.angle_min) / msg.angle_increment)
        start_index_2 = int((start_angle_2 - msg.angle_min) / msg.angle_increment)
        end_index_2 = int((end_angle_2 - msg.angle_min) / msg.angle_increment)

        start_index_1 = max(0, start_index_1)
        end_index_1 = min(len(msg.ranges), end_index_1)
        start_index_2 = max(0, start_index_2)
        end_index_2 = min(len(msg.ranges), end_index_2)

        filtered_ranges = [float('inf')] * len(msg.ranges)

        filtered_ranges[start_index_1:end_index_1] = msg.ranges[start_index_1:end_index_1]
        filtered_ranges[start_index_2:end_index_2] = msg.ranges[start_index_2:end_index_2]

        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.ranges = filtered_ranges
        filtered_msg.intensities = msg.intensities

        return filtered_msg

    def transform_scan(self, scan_msg, frame_id):
        try:
            transform = self.tf_buffer.lookup_transform('Mobile_Base', frame_id, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warn(f"Failed to find transform: {e}")
            return None

    def publish_combined_scan(self):
        if self.filtered_front_scan and self.filtered_back_scan:
            # ใช้ TF เพื่อหา position ของ front และ back scan
            front_transform = self.transform_scan(self.filtered_front_scan, 'lidar_front_link')
            back_transform = self.transform_scan(self.filtered_back_scan, 'lidar_back_link')

            if front_transform and back_transform:
                # รวมข้อมูล front และ back โดยใช้การแปลงจาก TF
                combined_msg = LaserScan()
                combined_msg.header = self.filtered_front_scan.header
                combined_msg.angle_min = self.filtered_front_scan.angle_min
                combined_msg.angle_max = self.filtered_back_scan.angle_max
                combined_msg.angle_increment = self.filtered_front_scan.angle_increment
                combined_msg.time_increment = self.filtered_front_scan.time_increment
                combined_msg.scan_time = self.filtered_front_scan.scan_time
                combined_msg.range_min = self.filtered_front_scan.range_min
                combined_msg.range_max = self.filtered_front_scan.range_max

                # Combine ranges from front and back (รักษาตำแหน่ง)
                combined_ranges = list(self.filtered_front_scan.ranges) + list(self.filtered_back_scan.ranges)
                combined_msg.ranges = combined_ranges

                combined_intensities = list(self.filtered_front_scan.intensities) + list(self.filtered_back_scan.intensities)
                combined_msg.intensities = combined_intensities

                self.filtered_combined_pub.publish(combined_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
