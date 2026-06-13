#!/usr/bin/env python3
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_parking_three_way_comparison():
    # File paths for all three datasets
    csv_file_40 = "/home/iwa/park_ws/src/lka_bringup/data/result_40deg_50gap.csv"
    csv_file_35 = "/home/iwa/park_ws/src/lka_bringup/data/result_35deg_50gap.csv"
    csv_file_30 = "/home/iwa/park_ws/src/lka_bringup/data/result_30deg_50gap.csv"
    
    print(f"Loading Dataset 1: {csv_file_40}")
    print(f"Loading Dataset 2: {csv_file_35}")
    print(f"Loading Dataset 3: {csv_file_30}")
    
    # Read all CSV dataframes
    df_40 = pd.read_csv(csv_file_40)
    df_35 = pd.read_csv(csv_file_35)
    df_30 = pd.read_csv(csv_file_30)
    
    # --- Convert Radians to Degrees for all datasets ---
    for df in [df_40, df_35, df_30]:
        df['Steering Command'] = np.degrees(df['Steering Command'])
        df['Yaw (deg)'] = np.degrees(df['Yaw (rad)'])
    # ----------------------------------------------------
    
    # Extract relative times from initial execution step for each dataset
    time_40 = df_40['Time (s)'] - df_40['Time (s)'].iloc[0]
    time_35 = df_35['Time (s)'] - df_35['Time (s)'].iloc[0]
    time_30 = df_30['Time (s)'] - df_30['Time (s)'].iloc[0]
    
    # Create subplots comparison canvas (2x2 grid)
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Parallel Parking Performance Comparison: 40° vs 35° vs 30° Max Steering Angle', fontsize=16, fontweight='bold')
    
    # Color and linestyle definitions to keep things consistent across plots
    c_40, ls_40 = 'blue', '-'
    c_35, ls_35 = 'red', '--'
    c_30, ls_30 = 'darkorange', '-.'
    
    # -------------------------------------------------------------
    # Plot 1: Overlaid Parking Trajectory (X position vs Y position)
    # -------------------------------------------------------------
    ax = axes[0, 0]
    ax.plot(df_40['X Position (m)'], df_40['Y Position (m)'], color=c_40, linestyle=ls_40, linewidth=2.5, label='40° Config (Solid Blue)')
    ax.plot(df_35['X Position (m)'], df_35['Y Position (m)'], color=c_35, linestyle=ls_35, linewidth=2.5, label='35° Config (Dashed Red)')
    ax.plot(df_30['X Position (m)'], df_30['Y Position (m)'], color=c_30, linestyle=ls_30, linewidth=2.5, label='30° Config (Dash-Dot Orange)')
    
    # Draw Start / End Markers
    ax.scatter(df_40['X Position (m)'].iloc[0], df_40['Y Position (m)'].iloc[0], c='green', s=120, marker='o', label='Start Position', zorder=5)
    ax.scatter(df_40['X Position (m)'].iloc[-1], df_40['Y Position (m)'].iloc[-1], c='blue', s=120, marker='X', label='40° End Point', zorder=5)
    ax.scatter(df_35['X Position (m)'].iloc[-1], df_35['Y Position (m)'].iloc[-1], c='red', s=120, marker='X', label='35° End Point', zorder=5)
    ax.scatter(df_30['X Position (m)'].iloc[-1], df_30['Y Position (m)'].iloc[-1], c='darkorange', s=120, marker='X', label='30° End Point', zorder=5)
    
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('Parking Trajectory Path Tracking')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')  # Enforces identical aspect scale ratio for X and Y parameters
    ax.legend(loc='best')
    
    # -------------------------------------------------------------
    # Plot 2: Steering Actuation Profile Over Time
    # -------------------------------------------------------------
    ax = axes[0, 1]
    ax.plot(time_40, df_40['Steering Command'], color=c_40, linestyle=ls_40, linewidth=2, label='40° Config')
    ax.plot(time_35, df_35['Steering Command'], color=c_35, linestyle=ls_35, linewidth=2, label='35° Config')
    ax.plot(time_30, df_30['Steering Command'], color=c_30, linestyle=ls_30, linewidth=2, label='30° Config')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Steering Command (deg)')
    ax.set_title('Steering Angle Actuation Profile')
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    
    # -------------------------------------------------------------
    # Plot 3: Vehicle Heading (Yaw Angle) Over Time
    # -------------------------------------------------------------
    ax = axes[1, 0]
    ax.plot(time_40, df_40['Yaw (deg)'], color=c_40, linestyle=ls_40, linewidth=2, label='40° Config')
    ax.plot(time_35, df_35['Yaw (deg)'], color=c_35, linestyle=ls_35, linewidth=2, label='35° Config')
    ax.plot(time_30, df_30['Yaw (deg)'], color=c_30, linestyle=ls_30, linewidth=2, label='30° Config')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw (deg)')
    ax.set_title('Vehicle Orientation Heading Angle ($\psi$)')
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    
    # -------------------------------------------------------------
    # Plot 4: Position Decomposition Component Comparison
    # -------------------------------------------------------------
    ax = axes[1, 1]
    # Blue shades for X Positions, Green shades for Y Positions
    ax.plot(time_40, df_40['X Position (m)'], 'b-', linewidth=2, label='40° - X Axis')
    ax.plot(time_40, df_40['Y Position (m)'], 'g-', linewidth=2, label='40° - Y Axis')
    
    ax.plot(time_35, df_35['X Position (m)'], 'cornflowerblue', linestyle='--', linewidth=1.5, label='35° - X Axis')
    ax.plot(time_35, df_35['Y Position (m)'], 'limegreen', linestyle='--', linewidth=1.5, label='35° - Y Axis')
    
    ax.plot(time_30, df_30['X Position (m)'], 'navy', linestyle=':', linewidth=1.5, label='30° - X Axis')
    ax.plot(time_30, df_30['Y Position (m)'], 'darkgreen', linestyle=':', linewidth=1.5, label='30° - Y Axis')
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Metrics (m)')
    ax.set_title('Decoupled Position Vectors Evolution')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', ncol=2)  # Expanded to 2 columns to keep layout clean
    
    plt.tight_layout()
    
    # Save figure plot directly into your bringup data path
    data_dir = "/home/iwa/park_ws/src/lka_bringup/data"
    output_file = os.path.join(data_dir, 'steering_three_way_comparison.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"\n[SUCCESS] Three-way comparison layout plots exported to: {output_file}")
    
    # Print cross-comparative dataset telemetry logs side-by-side in columns
    print(f"\n================================== PARKING PERFORMANCE STATISTICS ==================================")
    print(f"{'Maneuver Metric Evaluation':<32} | {'40° Config':<15} | {'35° Config':<15} | {'30° Config':<15}")
    print("-" * 100)
    print(f"{'Min Valid Space Limit ($L_{{min}}$)':<32} | {df_40['L_min (m)'].iloc[0]:.3f} m{'':<5} | {df_35['L_min (m)'].iloc[0]:.3f} m{'':<5} | {df_30['L_min (m)'].iloc[0]:.3f} m")
    print(f"{'Total Parking Duration Time':<32} | {time_40.iloc[-1]:.2f} s{'':<6} | {time_35.iloc[-1]:.2f} s{'':<6} | {time_30.iloc[-1]:.2f} s")
    print(f"{'Final Settling X Coordinate':<32} | {df_40['X Position (m)'].iloc[-1]:.3f} m{'':<5} | {df_35['X Position (m)'].iloc[-1]:.3f} m{'':<5} | {df_30['X Position (m)'].iloc[-1]:.3f} m")
    print(f"{'Final Settling Y Coordinate':<32} | {df_40['Y Position (m)'].iloc[-1]:.3f} m{'':<5} | {df_35['Y Position (m)'].iloc[-1]:.3f} m{'':<5} | {df_30['Y Position (m)'].iloc[-1]:.3f} m")
    print(f"{'Final Alignment Heading Yaw':<32} | {df_40['Yaw (deg)'].iloc[-1]:.3f} deg{'':<3} | {df_35['Yaw (deg)'].iloc[-1]:.3f} deg{'':<3} | {df_30['Yaw (deg)'].iloc[-1]:.3f} deg")
    print(f"{'Peak Absolute Steering Input':<32} | {max(abs(df_40['Steering Command'])):.2f} deg{'':<3} | {max(abs(df_35['Steering Command'])):.2f} deg{'':<3} | {max(abs(df_30['Steering Command'])):.2f} deg")
    print(f"====================================================================================================")

if __name__ == '__main__':
    plot_parking_three_way_comparison()