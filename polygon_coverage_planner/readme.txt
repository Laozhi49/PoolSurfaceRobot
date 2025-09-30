# 启动节点
ros2 run polygon_coverage_planner coverage_planner_action_server

ros2 launch polygon_coverage_planner polygon_coverage_planner.launch.py

# 测试
ros2 action send_goal /compute_polygon_coverage_path robot_interfaces/action/ComputePolygonCoveragePath "{polygon: [{x: 0.0, y: 0.0}, {x: 50.0, y: 10.0}, {x: 60.0, y: 50.0}, {x: 30.0, y: 60.0}, {x: 5.0, y: 40.0}]}"
