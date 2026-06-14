from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    L_arg = DeclareLaunchArgument(
        'L',
        default_value='6.7',
        description='Parking spot length (m)'
    )

    max_steer_arg = DeclareLaunchArgument(
        'max_steer',
        default_value='35.0',
        description='Maximum steering angle (deg)'
    )

    planning_node = Node(
        package='lka_bringup',
        executable='park_planning.py',
        name='parking_controller',
        output='screen',
        parameters=[{
            'L': LaunchConfiguration('L'),
            'max_steer': LaunchConfiguration('max_steer')
        }]
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
        L_arg,
        max_steer_arg,
        planning_node,
        log_node,
        plot_node
    ])