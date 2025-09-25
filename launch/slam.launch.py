# go2_nav2/launch/nav2_core_no_mapserver.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable   # ← 추가
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart    = LaunchConfiguration('autostart',    default='true')
    params_file  = LaunchConfiguration(
        'params_file',
        default=os.path.join(get_package_share_directory('go2_nav2'), 'params', 'nav2_params.yaml')
    )
    container_name = LaunchConfiguration('container_name', default='nav2_container')
    log_level      = LaunchConfiguration('log_level',      default='info')

    # ★ 이 한 줄로, 동일 프로세스에서 생성되는 모든 노드가 params_file을 참조
    set_param_env = SetEnvironmentVariable('RCL_YAML_PARAM_FILE', params_file)

    container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--params-file', params_file, '--log-level', log_level],
        composable_node_descriptions=[
            ComposableNode(package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
            ),
            ComposableNode(package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
            ),
            ComposableNode(package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
            ),
            ComposableNode(package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
            ),
            ComposableNode(package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
            ),
            ComposableNode(package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
            ),
            ComposableNode(package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
            ),
            ComposableNode(package='nav2_collision_monitor',
                plugin='nav2_collision_monitor::CollisionMonitor',
                name='collision_monitor',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
            ),
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lcl_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['amcl']
        }],
    )

    nav_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'planner_server',
                'smoother_server',
                'bt_navigator',
                'behavior_server',
                'waypoint_follower',
                'velocity_smoother',
                'collision_monitor'
            ]
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart',    default_value='true'),
        DeclareLaunchArgument('params_file',  default_value=params_file),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument('log_level',      default_value='info'),

        # ★ 환경변수를 컨테이너/노드들보다 먼저 적용
        set_param_env,

        container, amcl, lcl_mgr, nav_mgr
    ])
