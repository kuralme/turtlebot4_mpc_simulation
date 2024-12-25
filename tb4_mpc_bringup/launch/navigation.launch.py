import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    planner_config_dir = os.path.join(get_package_share_directory('path_planner_server'), 'config')
    map_config_dir = os.path.join(get_package_share_directory('map_server'), 'config')

    # Path Planner configurations
    planner_yaml = os.path.join(planner_config_dir, 'planner_server.yaml')
    default_bt_xml_path = os.path.join(planner_config_dir, 'behavior.xml')
    bt_navigator_yaml = os.path.join(planner_config_dir, 'bt_navigator.yaml')
    recovery_yaml = os.path.join(planner_config_dir, 'recovery.yaml')
    controller_yaml = os.path.join(planner_config_dir, 'controller.yaml')
    waypoint_follower_yaml = os.path.join(planner_config_dir, 'waypoint_follower.yaml')

    # Map filter configurations
    filters_yaml = os.path.join(map_config_dir, 'filters.yaml')
    map_keepout_yaml = os.path.join(map_config_dir, 'map_keepout.yaml')
    map_speeds_yaml = os.path.join(map_config_dir, 'map_speeds.yaml')
    
    # Localization configurations
    loc_yaml = os.path.join(get_package_share_directory(
        'tb4_mpc_bringup'), 'params', 'amcl.yaml')

    # Map & RViz configurations
    map_file = os.path.join(get_package_share_directory(
        'tb4_mpc_bringup'), 'maps', 'depot.yaml')
    rviz_config = os.path.join(get_package_share_directory(
        'tb4_mpc_bringup'), 'rviz', 'full_config.rviz')
    
    return LaunchDescription([
        Node(
            package='waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[waypoint_follower_yaml]),

        # Filter nodes
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml],
            remappings=[
                ('/keepout_filter_mask', map_keepout_yaml),
                ('/speed_filter_mask', map_speeds_yaml)]),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml]),
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': map_file}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[loc_yaml]
        ),

        # Controllers
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml]),
        # Node(
        #     package='mpc_controller',
        #     executable='controller_server',
        #     name='controller_server',
        #     output='screen',
        #     parameters=[controller_yaml]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[recovery_yaml]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml, {'default_bt_xml_filename': default_bt_xml_path}]),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz2_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config]),

        Node(
            package='tb4_mpc_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'controller_server',
                                        'planner_server',
                                        'recoveries_server',
                                        'bt_navigator',
                                        'waypoint_follower',
                                        'filter_mask_server',
                                        'costmap_filter_info_server']}])
    ])
