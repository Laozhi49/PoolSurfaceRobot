import math
import random
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


try:
    # ROS 2 提供的便捷创建 API（推荐）
    from sensor_msgs_py import point_cloud2
except Exception:
    point_cloud2 = None


def create_xyz32_cloud(header: Header, points: List[Tuple[float, float, float]]):
    """创建 xyz float32 的 PointCloud2。优先使用 sensor_msgs_py.point_cloud2。"""
    if point_cloud2 is not None:
        return point_cloud2.create_cloud_xyz32(header, points)
    # 兜底：手动构造（性能一般，但可用）
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = fields
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True
    import struct
    cloud.data = b''.join([struct.pack('<fff', *p) for p in points])
    return cloud

class OrbSlam3Sim(Node):
    def __init__(self):
        super().__init__('orb_slam3_sim_node')


        # 参数
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('cloud_rate_hz', 10.0)
        self.declare_parameter('cloud_size', 2000)
        self.declare_parameter('trajectory', 'eight') # 'eight' or 'circle'
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('height', 0.5)
        self.declare_parameter('noise_xyz', 0.01)


        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        cloud_rate_hz = self.get_parameter('cloud_rate_hz').get_parameter_value().double_value


        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )


        self.pose_pub = self.create_publisher(PoseStamped, '/orbslam3/camera_pose', qos)
        self.cloud_pub = self.create_publisher(PointCloud2, '/orbslam3/map_points', qos)


        self.t = 0.0
        self.dt = 1.0 / max(1e-3, rate_hz)
        self.cloud_dt = 1.0 / max(1e-3, cloud_rate_hz)
        self.last_cloud_pub_time = 0.0
        self.frame_id = frame_id


        self.timer = self.create_timer(self.dt, self.on_timer)
        self.get_logger().info('orb_slam3_sim_node started.')

    def on_timer(self):
        # 发布位姿
        pose = self.make_pose(self.t)
        self.pose_pub.publish(pose)

        # 控制点云发布频率
        if (self.t - self.last_cloud_pub_time) >= self.cloud_dt:
            cloud = self.make_cloud(self.t)
            self.cloud_pub.publish(cloud)
            self.last_cloud_pub_time = self.t
        
        self.t += self.dt

    # 轨迹：‘8’字或圆形
    def make_pose(self, t: float) -> PoseStamped:
        traj = self.get_parameter('trajectory').get_parameter_value().string_value
        R = self.get_parameter('radius').get_parameter_value().double_value
        z0 = self.get_parameter('height').get_parameter_value().double_value


        # if traj == 'eight':
        #     x = R * math.sin(t)
        #     y = R * math.sin(t) * math.cos(t)
        # else: # circle
        #     x = R * math.cos(t)
        #     y = R * math.sin(t)
        x = 0.5
        y = 0.0
        z = z0


        # 朝向：切向方向的 yaw
        # dx = -R * math.sin(t)
        # dy = R * math.cos(t)
        # yaw = math.atan2(dy, dx)
        yaw = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)


        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)
        return msg
    
    def make_cloud(self, t: float) -> PointCloud2:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        N = int(self.get_parameter('cloud_size').get_parameter_value().integer_value or 2000)
        noise = self.get_parameter('noise_xyz').get_parameter_value().double_value

        # 生成一个环绕原点的稀疏点云，并随时间做小幅旋转（近似地图点）
        points: List[Tuple[float, float, float]] = []
        random.seed(42) # 稳定可复现
        angle = 0.3 #* t
        ca, sa = math.cos(angle), math.sin(angle)


        for i in range(N):
            # 在若干个平面/体素附近采样，构成简单“房间”
            base = i % 5
            if base == 0: # 墙 1 (x ≈ +3)
                x, y, z = 3.0, (random.random()-0.5)*6.0, (random.random()-0.5)*2.5
            elif base == 1: # 墙 2 (x ≈ -3)
                x, y, z = -3.0, (random.random()-0.5)*6.0, (random.random()-0.5)*2.5
            elif base == 2: # 墙 3 (y ≈ +3)
                x, y, z = (random.random()-0.5)*6.0, 3.0, (random.random()-0.5)*2.5
            elif base == 3: # 墙 4 (y ≈ -3)
                x, y, z = (random.random()-0.5)*6.0, -3.0, (random.random()-0.5)*2.5
            else: # 地面/障碍 (z ≈ 0)
                x, y, z = (random.random()-0.5)*6.0, (random.random()-0.5)*6.0, 0.0

            # 绕 z 轴旋转一点点，让点云“动态”起来
            xr = ca * x - sa * y
            yr = sa * x + ca * y
            zr = z


            # 加噪
            xr += (random.random()-0.5) * 2.0 * noise
            yr += (random.random()-0.5) * 2.0 * noise
            zr += (random.random()-0.5) * 2.0 * noise


            points.append((xr, yr, zr))


        return create_xyz32_cloud(header, points)
    

def main():
    rclpy.init()
    node = OrbSlam3Sim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()