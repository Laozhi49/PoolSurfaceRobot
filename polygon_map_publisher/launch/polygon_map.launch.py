from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('polygon_map_publisher'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package="polygon_map_publisher",
            executable="polygon_map_node",
            name="polygon_map_node",
            output="screen",
            parameters=[config]
        )
    ])
