from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 参数文件路径
    config_path = PathJoinSubstitution([
        FindPackageShare('imu_processor'),
        'config',
        'imu_params.yaml'
    ])

    return LaunchDescription([
        # 加载YAML参数文件
        DeclareLaunchArgument(
            'config_file',
            default_value=config_path,
            description='Path to parameter config file'
        ),

        # 启动节点并加载参数
        Node(
            package='imu_processor',
            executable='imu_processor_node',
            name='imu_processor',
            parameters=[LaunchConfiguration('config_file')]
        )
    ])