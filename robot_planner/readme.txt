# 在新终端中运行
ros2 run robot_planner robot_planner_node

# 作为组件运行
ros2 component standalone robot_planner robot_planner::PlannerComponent

# 测试NavigateToPose
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 3.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
