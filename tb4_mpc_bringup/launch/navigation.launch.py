import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch_ros.actions import Node, SetParameter, PushROSNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('tb4_mpc_bringup')
    map_config_dir = os.path.join(get_package_share_directory('map_server'), 'config')
    filters_yaml = os.path.join(map_config_dir, 'filters.yaml')
    map_keepout_yaml = os.path.join(map_config_dir, 'map_keepout.yaml')
    map_speeds_yaml = os.path.join(map_config_dir, 'map_speeds.yaml')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    rviz_config_file = LaunchConfiguration('rviz_config')
    params_file = LaunchConfiguration('params_config')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {'autostart': autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # # Declare the launch arguments
    # declare_namespace_cmd = DeclareLaunchArgument(
    #     'namespace', 
    #     default_value='tb4', 
    #     description='Top-level namespace')
    
    # declare_map_yaml_file_cmd = DeclareLaunchArgument(
    #     'map', 
    #     default_value='', 
    #     description='Full path to map yaml file to load')

    # declare_rviz_config_file_cmd = DeclareLaunchArgument(
    #     'rviz_config', 
    #     default_value='', 
    #     description='Full path to rviz config file to load')

    # declare_use_sim_time_cmd = DeclareLaunchArgument(
    #     'use_sim_time',
    #     default_value='true',
    #     description='Use simulation (Gazebo) clock if true')

    # declare_params_file_cmd = DeclareLaunchArgument(
    #     'params_config',
    #     default_value=os.path.join(bringup_dir, 'params', 'tb4_nav2_mppi.yaml'),
    #     description='Full path to the ROS2 parameters file to use for all launched nodes')

    # declare_autostart_cmd = DeclareLaunchArgument(
    #     'autostart',
    #     default_value='true',
    #     description='Automatically startup the nav2 stack')

    # declare_use_respawn_cmd = DeclareLaunchArgument(
    #     'use_respawn',
    #     default_value='False',
    #     description='Whether to respawn if a node crashes. Applied when composition is disabled.')


    load_nodes = GroupAction(
        actions=[
            PushROSNamespace(namespace),
            SetParameter('use_sim_time', use_sim_time),

            # BT
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),

            # Localization
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),

            # Planning
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),
            
            # Controllers
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            # Node(
            #     package='nav2_smoother',
            #     executable='smoother_server',
            #     name='smoother_server',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params]),

            # Map & Filter nodes
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'yaml_filename': map_yaml_file}],
                remappings=remappings),
            # Node(
            #     package='nav2_map_server',
            #     executable='map_server',
            #     name='filter_mask_server',
            #     output='screen',
            #     emulate_tty=True,
            #     parameters=[filters_yaml],
            #     remappings=[
            #         ('/keepout_filter_mask', map_keepout_yaml),
            #         ('/speed_filter_mask', map_speeds_yaml)]),
            # Node(
            #     package='nav2_map_server',
            #     executable='costmap_filter_info_server',
            #     name='costmap_filter_info_server',
            #     output='screen',
            #     emulate_tty=True,
            #     parameters=[filters_yaml]),
            
            # Waypoint follower application
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),

            # Rviz
            Node(
                package='rviz2',
                executable='rviz2',
                namespace=namespace,
                arguments=['-d', rviz_config_file],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                remappings=remappings),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'autostart': autostart},
                            {'node_names': ['behavior_server',
                                            'bt_navigator',
                                            'amcl',
                                            'planner_server',
                                            'controller_server',
                                            'map_server',
                                            # 'costmap_filter_info_server',
                                            # 'filter_mask_server',
                                            'waypoint_follower',
                                            'rviz2'
                                            ]}
                            ])
        ]
    )
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    # ld.add_action(declare_namespace_cmd)
    # ld.add_action(declare_map_yaml_file_cmd)
    # ld.add_action(declare_rviz_config_file_cmd)
    # ld.add_action(declare_use_sim_time_cmd)
    # ld.add_action(declare_params_file_cmd)
    # ld.add_action(declare_autostart_cmd)
    # ld.add_action(declare_use_respawn_cmd)
    
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)

    return ld
