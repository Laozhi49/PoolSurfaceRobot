# 在新终端中运行
ros2 run robot_control robot_control_node


# 作为组件运行
ros2 component standalone robot_control robot_control::RobotControlComponent


# 终端1 - 启动组件容器
ros2 run rclcpp_components component_container_mt

# 终端2 - 加载组件
ros2 component load /ComponentManager robot_control robot_control::RobotControlComponent



# 测试rotate action
ros2 action send_goal /rotation robot_interfaces/action/Rotation "{target_angle: 30.0}" --feedback

# 测试move_distance action
ros2 action send_goal /move_distance robot_interfaces/action/MoveDistance "{distance: 2.0, speed: 0.5}" --feedback


# 调pid参数
ros2 param set /robot_control_node Kp 2.5
ros2 param set /robot_control_node Ki 0.1
ros2 param set /robot_control_node Kd 0.05

