
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 인자 선언
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration('map', default=os.path.join(
        get_package_share_directory('your_package_name'),
        'map', 'map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(
        get_package_share_directory('your_package_name'),
        'params', 'nav2_params.yaml'))
    autostart = LaunchConfiguration('autostart', default='true')

    bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=map_yaml),
        DeclareLaunchArgument('params_file', default_value=params_file),
        DeclareLaunchArgument('autostart', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml,
                'params_file': params_file,
                'autostart': autostart,
            }.items()
        )
    ])

# def generate_launch_description():
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     map_yaml_file = LaunchConfiguration('map')
#     bt_xml_file = LaunchConfiguration('bt_xml')

#     pkg_share = FindPackageShare('go2_nav2')

#     param_substitutions = {
#         'global_bt_xml_filename': PathJoinSubstitution([
#             pkg_share, 'params', 'behavior_tree', bt_xml_file
#         ])
#     }

#     configured_bt_navigator = RewrittenYaml(
#         source_file=PathJoinSubstitution([pkg_share, 'params', 'bt_navigator.yaml']),
#         root_key='bt_navigator',
#         param_rewrites=param_substitutions,
#         convert_types=True
#     )

#     return LaunchDescription([
#         DeclareLaunchArgument('use_sim_time', default_value='true'),
#         # DeclareLaunchArgument('map', default_value='map.yaml'),
#         # DeclareLaunchArgument('bt_xml', default_value='bt_tree.xml'),

#         # Node(
#         #     package='nav2_amcl',
#         #     executable='amcl',
#         #     name='amcl',
#         #     output='screen',
#         #     parameters=[
#         #         PathJoinSubstitution([pkg_share, 'params', 'amcl.yaml']),
#         #         {'use_sim_time': use_sim_time}
#         #     ]
#         # ),

#         # Node(
#         #     package='nav2_planner',
#         #     executable='planner_server',
#         #     name='planner_server',
#         #     output='screen',
#         #     parameters=[
#         #         PathJoinSubstitution([pkg_share, 'params', 'planner_server.yaml']),
#         #         PathJoinSubstitution([pkg_share, 'params', 'global_costmap.yaml']),
#         #         {'use_sim_time': use_sim_time}
#         #     ]
#         # ),

#         # Node(
#         #     package='nav2_controller',
#         #     executable='controller_server',
#         #     name='controller_server',
#         #     output='screen',
#         #     parameters=[
#         #         PathJoinSubstitution([pkg_share, 'params', 'controller_server.yaml']),
#         #         PathJoinSubstitution([pkg_share, 'params', 'local_costmap.yaml']),
#         #         PathJoinSubstitution([pkg_share, 'params', 'goal_checker.yaml']),  # ⬅️ selector가 정의돼 있으면
#         #         {'use_sim_time': use_sim_time}
#         #     ]
#         # ),

#         # Node(
#         #     package='nav2_behaviors',
#         #     executable='behavior_server',
#         #     name='behavior_server',
#         #     output='screen',
#         #     parameters=[
#         #         PathJoinSubstitution([pkg_share, 'params', 'behavior.yaml']),
#         #         {'use_sim_time': use_sim_time}
#         #     ]
#         # ),

#         # Node(
#         #     package='nav2_smoother',
#         #     executable='smoother_server',
#         #     name='smoother_server',
#         #     output='screen',
#         #     parameters=[
#         #         PathJoinSubstitution([pkg_share, 'params', 'smoother_server.yaml']),
#         #         {'use_sim_time': use_sim_time}
#         #     ]
#         # ),

#         # Node(
#         #     package='nav2_bt_navigator',
#         #     executable='bt_navigator',
#         #     name='bt_navigator',
#         #     output='screen',
#         #     parameters=[
#         #         configured_bt_navigator,
#         #         PathJoinSubstitution([pkg_share, 'params', 'goal_checker_selector.yaml']),  # ✅ 반드시 추가
#         #         {'use_sim_time': use_sim_time}
#         #     ]
#         # ),

#         # Node(
#         #     package='nav2_velocity_smoother',
#         #     executable='velocity_smoother',
#         #     name='velocity_smoother',
#         #     output='screen',
#         #     parameters=[
#         #         PathJoinSubstitution([pkg_share, 'params', 'velocity_smoother.yaml']),
#         #         {'use_sim_time': use_sim_time}
#         #     ]
#         # ),



#         Node(
#             package='nav2_lifecycle_manager',
#             executable='lifecycle_manager',
#             name='lifecycle_manager',
#             output='screen',
#             parameters=[
#                 {'use_sim_time': use_sim_time},
#                 {'autostart': True},
#                 {'node_names': [
#                     'amcl',
#                     'planner_server',
#                     'controller_server',
#                     'bt_navigator',
#                     'behavior_server',
#                     'smoother_server',
#                     'velocity_smoother'
#                 ]}
#             ]
#         )
#     ])
