from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='commander_app',
            executable='follow_waypoints',
            name='commander_app',
            output='screen',
            parameters=[{
                'use_sim_time': True  # Set to False if not using simulation time
            }]
        )
    ])