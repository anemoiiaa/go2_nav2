#!/usr/bin/env python3
# imu_ekf_bridge.launch.py
# - EKF(robot_localization)만 필수로 띄움
# - map→odom 브릿지(MapOdomBridge)는 옵션(use_bridge)으로 켜고/끄기
# - ekf_params는 절대경로로 넘겨 안정적으로 파라미터 로딩

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable   # ← 추가
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # ---- Launch args ----
    config_file  = LaunchConfiguration(
        'params_file',
        default=os.path.join(get_package_share_directory('go2_nav2'), 'config', 'ekf_local.yaml')
    )
    use_bridge = DeclareLaunchArgument(
        'use_bridge', default_value='true',
        description='Whether to run map→odom bridge node.'
    )
    base_frame = DeclareLaunchArgument('base_frame', default_value='base_link')
    map_frame  = DeclareLaunchArgument('map_frame',  default_value='map')
    odom_frame = DeclareLaunchArgument('odom_frame', default_value='odom')
    global_odom_topic = DeclareLaunchArgument(
        'global_odom_topic',
        default_value='/lio_sam/mapping/odometry',
        description='LIO-SAM global odometry topic (frame_id=map, child=base_link).'
    )
    ekf_node_name = DeclareLaunchArgument('ekf_node_name', default_value='ekf_local')

    # ---- Nodes ----
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[config_file],
    )

    # # 브릿지 노드: 네가 만든/담은 패키지명과 실행명에 맞춰주세요.
    # # 예: package='go2_nav2', executable='bridge_map_odom'
    # bridge_node = Node(
    #     package='go2_nav2',                  # ← 너의 패키지명으로
    #     executable='bridge_map_odom',        # ← console_scripts로 등록한 실행명
    #     name='map_odom_bridge',
    #     output='screen',
    #     parameters=[{
    #         'base_frame': LaunchConfiguration('base_frame'),
    #         'map_frame':  LaunchConfiguration('map_frame'),
    #         'odom_frame': LaunchConfiguration('odom_frame'),
    #         'global_odom_topic': LaunchConfiguration('global_odom_topic'),
    #     }],
    #     condition=IfCondition(LaunchConfiguration('use_bridge')),
    # )

    return LaunchDescription([
        # ekf_params,use_bridge, base_frame, map_frame, odom_frame,
        # global_odom_topic, ekf_node_name,
        ekf_node,
        # bridge_node,
    ])
