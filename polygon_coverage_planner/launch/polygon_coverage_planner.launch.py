from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('polygon_coverage_planner'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package="polygon_coverage_planner",
            executable="coverage_planner_action_server",
            name="coverage_planner_action_server",
            output="screen",
            parameters=[config]
        )
    ])
