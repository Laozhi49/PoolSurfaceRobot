from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import LogInfo


def generate_launch_description():
    # 参数文件
    config_file = os.path.join(
        get_package_share_directory('imu_processor'),  # 你的 launch 包名
        'config',
        'imu_params.yaml'
    )

    LogInfo(msg=f"Loading config file: {config_file}")
    
    return LaunchDescription([
        # 启动节点并加载参数
        Node(
            package='imu_processor',
            executable='imu_processor_node',
            name='imu_processor',
            parameters=[config_file]
        )
    ])