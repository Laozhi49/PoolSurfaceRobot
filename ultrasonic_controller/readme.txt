# 查看节点状态
ros2 lifecycle list /ultrasonic_controller

# 配置节点
ros2 lifecycle set /ultrasonic_controller configure

# 激活节点
ros2 lifecycle set /ultrasonic_controller activate

# 停用节点
ros2 lifecycle set /ultrasonic_controller deactivate

# 关闭节点
ros2 lifecycle set /ultrasonic_controller shutdown

# 通过launch启动
ros2 launch ultrasonic_controller ultrasonic_controller.launch.py