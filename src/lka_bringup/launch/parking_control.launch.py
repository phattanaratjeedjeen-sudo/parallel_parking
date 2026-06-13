from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    planning_node = Node(
        package='lka_bringup',
        executable='park_planning.py',
        name='parking_controller',
        output='screen'
    )
    
    log_node = Node(
        package='lka_bringup',
        executable='log_data.py',
        name='result_recorder',
        output='screen'
    )

    plot_node = Node(
        package='lka_bringup',
        executable='plot_results.py',
        name='result_plotter',
        output='screen'
    )
    
    return LaunchDescription([
        planning_node,
        log_node,
        plot_node
    ])