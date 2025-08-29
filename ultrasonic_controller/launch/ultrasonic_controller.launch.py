from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取参数文件路径
    config = os.path.join(
        get_package_share_directory('ultrasonic_controller'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        LifecycleNode(
            package='ultrasonic_controller',
            executable='ultrasonic_controller',
            name='ultrasonic_controller',
            namespace='',
            output='screen',
            parameters=[config]
        )
    ])