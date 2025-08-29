from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='robot_costmap',
            executable='robot_costmap_node',
            name='robot_costmap_component',
            namespace='',
            parameters=['install/robot_costmap/share/robot_costmap/config/robot_costmap_params.yaml'],
            output='screen'
        )
    ])
