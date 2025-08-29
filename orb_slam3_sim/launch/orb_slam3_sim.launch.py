from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('rate_hz', default_value='30.0'),
        DeclareLaunchArgument('cloud_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('cloud_size', default_value='2000'),
        DeclareLaunchArgument('trajectory', default_value='eight'), # 'eight' or 'circle'
        DeclareLaunchArgument('radius', default_value='2.0'),
        DeclareLaunchArgument('height', default_value='0.5'),
        DeclareLaunchArgument('noise_xyz', default_value='0.01'),


        Node(
            package='orb_slam3_sim',
            executable='orb_slam3_sim_node',
            name='orb_slam3_sim_node',
            output='screen',
            parameters=[
                {
                    'frame_id': LaunchConfiguration('frame_id'),
                    'rate_hz': LaunchConfiguration('rate_hz'),
                    'cloud_rate_hz': LaunchConfiguration('cloud_rate_hz'),
                    'cloud_size': LaunchConfiguration('cloud_size'),
                    'trajectory': LaunchConfiguration('trajectory'),
                    'radius': LaunchConfiguration('radius'),
                    'height': LaunchConfiguration('height'),
                    'noise_xyz': LaunchConfiguration('noise_xyz'),
                }
            ]
        )
    ])