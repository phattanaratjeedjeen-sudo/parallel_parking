#!/usr/bin/env python3
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_parking_results(csv_file=None):

    data_dir = os.path.expanduser('~/park_ws/src/lka_bringup/data')
    
    # # Find CSV file
    # if csv_file is None:
    #     csv_files = sorted(glob.glob(os.path.join(data_dir, 'result_*.csv')))
    #     if not csv_files:
    #         print(f"No result_*.csv files found in {data_dir}")
    #         return
    #     csv_file = csv_files[-1]  # Use latest file
    
    csv_file = "/home/iwa/park_ws/src/lka_bringup/data/result_61.csv"
    print(f"Loading data from: {csv_file}")
    
    # Read CSV
    df = pd.read_csv(csv_file)
    
    # --- Convert Radians to Degrees ---
    # Convert Steering and Yaw columns to degrees
    df['Steering Command'] = np.degrees(df['Steering Command'])
    df['Yaw (deg)'] = np.degrees(df['Yaw (rad)'])
    # ----------------------------------
    
    # Extract filename without extension for output naming
    base_name = os.path.splitext(os.path.basename(csv_file))[0]
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'Parking Maneuver Results - {base_name}', fontsize=16)
    
    # Plot 1: Trajectory (X vs Y)
    ax = axes[0, 0]
    ax.plot(df['X Position (m)'], df['Y Position (m)'], 'b-', linewidth=2, label='Car Path')
    ax.scatter(df['X Position (m)'].iloc[0], df['Y Position (m)'].iloc[0], 
               c='green', s=100, marker='o', label='Spawn', zorder=5)
    ax.scatter(df['X Position (m)'].iloc[-1], df['Y Position (m)'].iloc[-1], 
               c='red', s=100, marker='X', label='End', zorder=5)
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('Parking Trajectory')
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
    
    # Plot 3: Y Position vs Time
    ax = axes[1, 0]
    ax.plot(time, df['Y Position (m)'], 'g-', linewidth=2, label='Y Position')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('Y Position Over Time')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # Plot 4: X Position vs Time
    ax = axes[1, 1]
    ax.plot(time, df['X Position (m)'], 'b-', linewidth=2, label='X Position')
    ax.axhline(y=71.2, color='orange', linestyle='--', linewidth=2, label='Upper Limit (71.2)')
    ax.axhline(y=69.8, color='orange', linestyle='--', linewidth=2, label='Lower Limit (69.8)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('X Position (m)')
    ax.set_title('X Position Over Time')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    plt.tight_layout()
    
    # Save figure
    output_file = os.path.join(data_dir, f'{base_name}_plot.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    
    # Print statistics
    print(f"\n=== Parking Maneuver Statistics ===")
    print(f"L_min: {calculate_l_min(df['Steering Command'].max()):.2f} m")
    print(f"Total time: {time.iloc[-1]:.2f} s")
    print(f"Start position: ({df['X Position (m)'].iloc[0]:.2f}, {df['Y Position (m)'].iloc[0]:.2f})")
    print(f"End position: ({df['X Position (m)'].iloc[-1]:.2f}, {df['Y Position (m)'].iloc[-1]:.2f})")
    print(f"Start yaw: {df['Yaw (deg)'].iloc[0]:.2f} deg")
    print(f"End yaw: {df['Yaw (deg)'].iloc[-1]:.2f} deg")
    print(f"Max steering: {df['Steering Command'].max():.2f} deg")
    print(f"Min steering: {df['Steering Command'].min():.2f} deg")
    
    plt.show()


def calculate_l_min(steer_max):
        a = 3.005
        b = 1.67 / 2.0
        d_front = 0.81
        d_rear = 0.98
        d_side = 0.25
        
        RE_min = a / np.tan(np.deg2rad(steer_max))
        R_Bl_min = np.sqrt((RE_min + b + d_side)**2 + (a + d_front)**2)
        L_min = d_rear + np.sqrt(R_Bl_min**2 - (RE_min - b - d_side)**2)
        
        return L_min

if __name__ == '__main__':
    import sys
    csv_file = sys.argv[1] if len(sys.argv) > 1 else None
    plot_parking_results(csv_file)