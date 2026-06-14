#!/usr/bin/env python3

import json
import csv
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

class ResultRecorder(Node):
    def __init__(self):
        super().__init__('result_recorder')        
        self.data_records = []
        
        self.L = None
        self.max_steer = None
        self.current_gear_changes = 0

        self.data_dir = os.path.expanduser('~/park_ws/src/lka_bringup/data/results/csv')
        os.makedirs(self.data_dir, exist_ok=True)
        self.csv_file = None
        self.csv_writer = None
        self.csv_file_handle = None
        self.latest_control = None
        self.latest_pose = None

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

        self.gear_sub = self.create_subscription(
            Int32,
            '/carla/ego_vehicle/gear_changes',
            self.gear_callback,
            10
        )
        
        self.config_client = self.create_client(
            Trigger, 
            '/carla/ego_vehicle/get_park_config'
        )
        self.timer = self.create_timer(1.0, self.request_config)

    def gear_callback(self, msg: Int32):
        self.current_gear_changes = msg.data
        
    def request_config(self):
        if self.csv_file is not None:
            self.timer.cancel()
            return
            
        if self.config_client.wait_for_service(timeout_sec=0.1):
            req = Trigger.Request()
            future = self.config_client.call_async(req)
            future.add_done_callback(self.config_response_callback)
            
    def config_response_callback(self, future):
        if self.csv_file is not None:
            return
        try:
            response = future.result()
            if response.success:
                config = json.loads(response.message)
                self.L = float(config.get("L"))
                self.max_steer = float(config.get("max_steer"))
                self.csv_file = self.get_next_csv_filename()
                self.initialize_csv()
                self.timer.cancel()
        except Exception as e:
            self.get_logger().error(f"Failed to parse config from service: {e}")

    def get_next_csv_filename(self):
        index = 1
        while True:
            filename = os.path.join(self.data_dir, f'park_{self.L}_{self.max_steer}_{index}.csv')
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
            'change_gear_times'
        ])
        self.csv_file_handle.flush()
        self.get_logger().info(f'CSV file initialized at {self.csv_file}')
        
    
    def control_callback(self, msg: CarlaEgoVehicleControl):
        """Store the latest control command"""
        self.latest_control = msg
        
    def odom_callback(self, msg: Odometry):
        """Callback for odometry data"""
        if self.csv_file is None:
            return
            
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
            f'{self.current_gear_changes}'
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
