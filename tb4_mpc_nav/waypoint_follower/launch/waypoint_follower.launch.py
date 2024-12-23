from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_follower',
            executable='follow_waypoints',
            name='waypoint_follower',
            output='screen',
            parameters=[{
                'use_sim_time': True  # Set to False if not using simulation time
            }]
        )
    ])