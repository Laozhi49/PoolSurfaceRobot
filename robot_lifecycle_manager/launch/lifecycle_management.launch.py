from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # 参数文件路径
    config_file = os.path.join(
        get_package_share_directory('robot_lifecycle_manager'),  # 你的 launch 包名
        'config',
        'robot_params.yaml'
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_description'),
                'launch',
                'robot_description.launch.py'
            )
        )
    )

    coverage_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('polygon_coverage_planner'),
                'launch',
                'polygon_coverage_planner.launch.py'
            )
        )
    )

    polygon_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('polygon_map_publisher'),
                'launch',
                'polygon_map.launch.py'
            )
        )
    )

    container = ComposableNodeContainer(
        name='lifecycle_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='robot_control',
                plugin='robot_control::RobotControlComponent',
                name='robot_control',
                namespace='',
                parameters=[config_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='pointcloud_to_2dmap',
                plugin='pointcloud_to_2dmap::PointCloudTo2DMap',
                name='pointcloud_to_2dmap',
                namespace='',
                parameters=[config_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='robot_costmap',
                plugin='robot_costmap::RobotCostmapComponent',
                name='robot_costmap',
                namespace='',
                parameters=[config_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='robot_planner',
                plugin='robot_planner::PlannerComponent',
                name='robot_planner',
                namespace='',
                parameters=[config_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='robot_behavior',
                plugin='robot_behavior::BehaviorTreeComponent',
                name='robot_behavior',
                namespace='',
                parameters=[config_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # 生命周期管理器（必须最后加载）
            ComposableNode(
                package='robot_lifecycle_manager',
                plugin='robot_lifecycle_manager::LifecycleManager',
                name='robot_lifecycle_manager',
                parameters=[config_file],
                namespace='',
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_description_launch,
        coverage_planner_launch,
        container,
        polygon_map_launch
    ])