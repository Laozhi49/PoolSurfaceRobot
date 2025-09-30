import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from robot_interfaces.action import ComputePolygonCoveragePath
from polygon_coverage_planner.coverage_planner import generate_coverage_path

class CoveragePlannerActionServer(Node):
    def __init__(self):
        super().__init__('coverage_planner_action_server')

        self.declare_parameter('robot_radius', 0.3)
        self.declare_parameter('step_size', 0.45)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("coverage_planner_action_name", "compute_polygon_coverage_path")
        self.declare_parameter("coverage_path_topic", "coverage_path_marker")

        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.step_size = self.get_parameter('step_size').get_parameter_value().double_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.coverage_planner_action_name = self.get_parameter("coverage_planner_action_name").get_parameter_value().string_value
        self.coverage_path_topic = self.get_parameter("coverage_path_topic").get_parameter_value().string_value

        self._action_server = ActionServer(
            self,
            ComputePolygonCoveragePath,
            self.coverage_planner_action_name,
            self.execute_callback)
        
        # 发布 Marker（RViz2 可视化）
        self.marker_pub_ = self.create_publisher(Marker, self.coverage_path_topic, 10)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal request')

        polygon = [(p.x, p.y) for p in goal_handle.request.polygon]

        # 内部固定参数
        robot_radius = self.robot_radius
        step_size = self.step_size

        path_points = generate_coverage_path(polygon, robot_radius, step_size)

        result = ComputePolygonCoveragePath.Result()
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in path_points:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        result.path = path_msg
        self.publish_marker(path_msg)

        goal_handle.succeed()
        return result

    def publish_marker(self, path: Path):
        marker = Marker()
        marker.header = path.header
        marker.ns = "coverage_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05  # 线宽
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for pose_stamped in path.poses:
            marker.points.append(pose_stamped.pose.position)

        self.marker_pub_.publish(marker)
        
def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlannerActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
