########################### 启动MicroRos #####################################
# 启动Agent
sudo chmod 777 /dev/ttyACM0

ros2 run micro_ros_agent micro_ros_agent serial -b 1000000 --dev /dev/ttyACM0


################################################################################
# PoolSurfaceRobot

# 启动组件管理器
ros2 launch robot_lifecycle_manager lifecycle_management.launch.py

# 启动imu
ros2 launch imu_processor imu_processor.launch.py

# 启动摄像头
ros2 launch stereo_camera stereo_camera.launch.py


# to do:
    自主探索建图/沿边走探索建图

    不规则地图的路径覆盖算法

    完善导航功能，比如导航过程中遇到困难时脱困

