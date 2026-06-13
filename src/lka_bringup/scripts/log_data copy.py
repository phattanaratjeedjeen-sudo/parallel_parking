#!/usr/bin/env python3

import csv
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
from tf_transformations import euler_from_quaternion

class ResultRecorder(Node):
    def __init__(self):
        super().__init__('result_recorder')        
        self.data_records = []
        
        self.data_dir = os.path.expanduser('~/park_ws/src/lka_bringup/data')
        os.makedirs(self.data_dir, exist_ok=True)
        self.csv_file = self.get_next_csv_filename()
        self.csv_writer = None
        self.csv_file_handle = None
        self.initialize_csv()

        self.odom_sub = self.create_subscription(
            Odometry, 
            '/carla/ego_vehicle/odometry', 
            self.odom_callback, 
            10
        )
        
        self.control_sub = self.create_subscription(
            CarlaEgoVehicleControl,
            '/carla/ego_vehicle/vehicle_control_cmd',
            self.control_callback,
            10
        )
        
        self.latest_control = None
        self.latest_pose = None
        
        self.get_logger().info('ResultRecorder node initialized')
    
    def get_next_csv_filename(self):
        """Generate next available CSV filename (result_1.csv, result_2.csv, etc.)"""
        index = 1
        while True:
            filename = os.path.join(self.data_dir, f'result_{index}.csv')
            if not os.path.exists(filename):
                return filename
            index += 1
        
    def initialize_csv(self):
        """Initialize CSV file with headers"""
        self.csv_file_handle = open(self.csv_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file_handle)
        self.csv_writer.writerow([
            'Timestamp (ns)',
            'Time (s)',
            'X Position (m)',
            'Y Position (m)',
            'Yaw (rad)',
            'Steering Command',
        ])
        self.csv_file_handle.flush()
        self.get_logger().info(f'CSV file initialized at {self.csv_file}')
        
    
    def control_callback(self, msg: CarlaEgoVehicleControl):
        """Store the latest control command"""
        self.latest_control = msg
        
    def odom_callback(self, msg: Odometry):
        """Callback for odometry data"""
        # Extract pose
        q = msg.pose.pose.orientation
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # Get timestamp
        timestamp_ns = msg.header.stamp.sec * int(1e9) + msg.header.stamp.nanosec
        time_s = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
    
        steer = 0.0
        if self.latest_control:
            steer = self.latest_control.steer
        
        self.csv_writer.writerow([
            timestamp_ns,
            f'{time_s:.4f}',
            f'{x:.6f}',
            f'{y:.6f}',
            f'{yaw:.6f}',
            f'{steer:.6f}',
        ])
        self.csv_file_handle.flush()
        
    def destroy_node(self):
        if self.csv_file_handle:
            self.csv_file_handle.close()
            self.get_logger().info(f'CSV file saved to {self.csv_file}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ResultRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
