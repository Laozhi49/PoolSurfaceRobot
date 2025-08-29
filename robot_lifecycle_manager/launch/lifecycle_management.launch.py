from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
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
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='pointcloud_to_2dmap',
                plugin='pointcloud_to_2dmap::PointCloudTo2DMap',
                name='pointcloud_to_2dmap',
                namespace='',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='robot_costmap',
                plugin='robot_costmap::RobotCostmapComponent',
                name='robot_costmap',
                namespace='',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='robot_planner',
                plugin='robot_planner::PlannerComponent',
                name='robot_planner',
                namespace='',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='robot_behavior',
                plugin='robot_behavior::BehaviorTreeComponent',
                name='robot_behavior',
                namespace='',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # 生命周期管理器（必须最后加载）
            ComposableNode(
                package='robot_lifecycle_manager',
                plugin='robot_lifecycle_manager::LifecycleManager',
                name='robot_lifecycle_manager',
                parameters=[{
                    'managed_nodes': ['/robot_control','/pointcloud_to_2dmap','/robot_behavior','/robot_costmap','/robot_planner']
                }],
                namespace='',
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen'
    )

    return LaunchDescription([container])