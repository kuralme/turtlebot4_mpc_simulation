"""Parent launch script for simulation and navigation stacks."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get project directories
    bringup_dir = get_package_share_directory('tb4_mpc_bringup')
    sim_dir = get_package_share_directory('tb4_mpc_sim')
    map_config_dir = os.path.join(bringup_dir, 'maps', 'depot.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory(
        'tb4_mpc_bringup'), 'rviz', 'depot_config.rviz')
    nav_config = os.path.join(bringup_dir, 'params', 'tb4_nav2_mppi.yaml')
    
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', 
        default_value='', 
        # default_value='tb4', 
        description='Top-level namespace')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', 
        default_value=map_config_dir, 
        description='Full path to map yaml file to load')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config', 
        default_value=rviz_config_dir, 
        description='Full path to rviz config file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='true',
        description='Whether to start the robot state publisher')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav_config,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')


    # Launch Gazebo simulation with robot in the depot world
    tb4_sim_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'depot_world_tb4.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'use_robot_state_pub': use_robot_state_pub,
        }.items()
    )

    # Launch Navigation2 stack
    tb4_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'map': map_yaml_file,
            'params_config': params_file,
            'rviz_config': rviz_config_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)

    ld.add_action(tb4_sim_world)
    ld.add_action(tb4_navigation)

    return ld
