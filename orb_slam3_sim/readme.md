# orb_slam3_sim


模拟 ORB‑SLAM3 输出：`/orb_slam3/pose` (PoseStamped) 与 `/orb_slam3/points` (PointCloud2)。


## 编译
```bash
# 在你的 ROS 2 工作空间 src/ 下
cd ~/ros2_ws/src
# 拷贝或创建本包
# 然后返回工作空间根目录
cd ..
colcon build --packages-select orb_slam3_sim
source install/setup.bash
```

## 启动
```bash
ros2 launch orb_slam3_sim orb_slam3_sim.launch.py \
frame_id:=map rate_hz:=30.0 cloud_rate_hz:=10.0 \
cloud_size:=2000 trajectory:=eight radius:=2.0 height:=0.5 noise_xyz:=0.01

```

## 可视化（RViz2）：

Fixed Frame 设为 map

显示 /orb_slam3/pose（Pose）和 /orb_slam3/points（PointCloud2）