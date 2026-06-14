#!/usr/bin/env python3
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np
import json

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class PlotResultsNode(Node):
    def __init__(self):
        super().__init__('result_plotter')
        self.csv_dir = os.path.expanduser('~/park_ws/src/lka_bringup/data/results/csv')
        self.plot_dir = os.path.expanduser('~/park_ws/src/lka_bringup/data/results/plot')
        os.makedirs(self.plot_dir, exist_ok=True)
        
        self.plot_srv = self.create_service(Trigger, '/carla/ego_vehicle/plot_results', self.plot_request_callback)
        self.config_client = self.create_client(Trigger, '/carla/ego_vehicle/get_park_config')

    def plot_request_callback(self, request, response):
        if self.config_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            future = self.config_client.call_async(req)
            future.add_done_callback(self.config_response_callback)
            response.success = True
            response.message = "Plot generation started."
        else:
            self.get_logger().error('Config service not available')
            response.success = False
            response.message = "Config service not available."
        return response
        
    def config_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                config = json.loads(response.message)
                car_length = float(config.get("car_length", 4.795))
                self.plot_parking_results(car_length)
        except Exception as e:
            self.get_logger().error(f"Failed to get config from service: {e}")
        finally:
            self.create_timer(1.0, self.shutdown_node)

    def shutdown_node(self):
        rclpy.shutdown()

    def plot_parking_results(self, car_length):
        # Find the latest CSV file in the results/csv directory
        csv_files = sorted(glob.glob(os.path.join(self.csv_dir, '*.csv')), key=os.path.getmtime)
        if not csv_files:
            self.get_logger().error(f"No CSV files found in {self.csv_dir}")
            return
        
        csv_file = csv_files[-1]
        self.get_logger().info(f"Loading data from: {csv_file}")
        
        # Read CSV
        df = pd.read_csv(csv_file)
        
        # Convert Radians to Degrees
        df['Steering Command'] = np.degrees(df['Steering Command'])
        df['Yaw (deg)'] = np.degrees(df['Yaw (rad)'])
        
        # Extract filename without extension for output naming
        base_name = os.path.splitext(os.path.basename(csv_file))[0]
        
        # Create figure with subplots (3 rows, 2 columns to fit 5 plots)
        fig, axes = plt.subplots(3, 2, figsize=(14, 15))
        fig.suptitle(f'Parking Maneuver Results - {base_name}', fontsize=16)
        
        # Plot 1: Trajectory (X vs Y)
        ax = axes[0, 0]
        ax.plot(df['X Position (m)'], df['Y Position (m)'], 'b-', linewidth=2, label='Car Path')
        ax.scatter(df['X Position (m)'].iloc[0], df['Y Position (m)'].iloc[0], 
                   c='green', s=100, marker='o', label='Spawn', zorder=5)
        ax.scatter(df['X Position (m)'].iloc[-1], df['Y Position (m)'].iloc[-1], 
                   c='red', s=100, marker='X', label='End', zorder=5)
        
        # Find when the parking maneuver actually starts (first steering command)
        start_mask = df['Steering Command'].abs() > 0.001
        if start_mask.any():
            start_idx = start_mask.idxmax()
            ax.scatter(df['X Position (m)'].iloc[start_idx], df['Y Position (m)'].iloc[start_idx], 
                       c='orange', s=100, marker='D', label='Start Planning', zorder=5)
        
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_title('Parking Trajectory')
        
        final_x = df['X Position (m)'].iloc[-1]
        final_y = df['Y Position (m)'].iloc[-1]
        ax.text(0.05, 0.85, f'Final: X={final_x:.2f}, Y={final_y:.2f}',
                transform=ax.transAxes, fontsize=11, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='b', alpha=0.8))
        
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.axis('equal')
        
        # Plot 2: Steering Command vs Time (Converted to deg)
        ax = axes[0, 1]
        time = df['Time (s)'] - df['Time (s)'].iloc[0]  # Relative time
        ax.plot(time, df['Steering Command'], 'r-', linewidth=2)
        ax.fill_between(time, df['Steering Command'], alpha=0.3, color='red')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Steering Command (deg)')
        ax.set_title('Steering Command Over Time')
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)

        # Plot 3: X Position vs Time
        ax = axes[1, 0]
        ax.plot(time, df['X Position (m)'], 'b-', linewidth=2, label='X Position')
        
        L = 6.2  # Default fallback
        try:
            parts = base_name.split('_')
            if len(parts) >= 3 and parts[0] == 'park':
                L = float(parts[1])
        except ValueError:
            pass
            
        lower_limit = 69.8
        upper_limit = lower_limit + L - car_length
        
        ax.axhline(y=upper_limit, color='orange', linestyle='--', linewidth=2, label=f'Upper Limit ({upper_limit:.2f})')
        ax.axhline(y=lower_limit, color='orange', linestyle='--', linewidth=2, label=f'Lower Limit ({lower_limit:.2f})')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('X Position (m)')
        ax.set_title('X Position Over Time')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # Plot 4: Y Position vs Time
        ax = axes[1, 1]
        ax.plot(time, df['Y Position (m)'], 'g-', linewidth=2, label='Y Position')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Y Position (m)')
        ax.set_title('Y Position Over Time')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # Plot 5: Gear Changes vs Time
        if 'change_gear_times' in df.columns:
            ax = axes[2, 0]
            gear_changes = df['change_gear_times']
            ax.plot(time, gear_changes, 'm-', linewidth=2, drawstyle='steps-post', label='Gear Changes')
            
            total_gears = int(gear_changes.iloc[-1])
            ax.text(0.05, 0.85, f'Total Gear Changes: {total_gears}',
                    transform=ax.transAxes, fontsize=11, fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='m', alpha=0.8))
            
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Gear Changes')
            ax.set_title('Numbers of Gear Changing')
            ax.yaxis.set_major_locator(MaxNLocator(integer=True))
            ax.grid(True, alpha=0.3)
            ax.legend()
            
        # Plot 6: Yaw Position vs Time
        ax = axes[2, 1]
        ax.plot(time, df['Yaw (deg)'], 'c-', linewidth=2, label='Yaw')
        
        final_yaw = df['Yaw (deg)'].iloc[-1]
        ax.text(0.05, 0.85, f'Final Yaw: {final_yaw:.2f}°',
                transform=ax.transAxes, fontsize=11, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='c', alpha=0.8))
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Yaw (deg)')
        ax.set_title('Yaw Over Time')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        plt.tight_layout()
        
        # Save figure in the target plot directory
        output_file = os.path.join(self.plot_dir, f'{base_name}.png')
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        self.get_logger().info(f"Plot successfully saved to: {output_file}")


def main(args=None):
    rclpy.init(args=args)
    node = PlotResultsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()