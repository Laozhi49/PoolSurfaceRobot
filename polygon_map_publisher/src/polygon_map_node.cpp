#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <vector>
#include <cmath>

using std::placeholders::_1;

geometry_msgs::msg::Point make_point(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

class PolygonMapNode : public rclcpp::Node
{
public:
  PolygonMapNode()
  : Node("polygon_map_node")
  {
    // 参数：地图大小和分辨率
    declare_parameter("map_width", 100);   // 单位：cell
    declare_parameter("map_height", 100);  // 单位：cell
    declare_parameter("resolution", 0.05);  // 每个cell大小 (m)

    int width = get_parameter("map_width").as_int();
    int height = get_parameter("map_height").as_int();
    double res = get_parameter("resolution").as_double();

    // 设置固定的多边形顶点 (单位: m)
    // 例如一个矩形
    polygon_ = {
        make_point(-2.0, 0.0, 0.0),
        make_point(-2.0, 2.11, 0.0),
        make_point(2.0, 2.11, 0.0),
        make_point(2.0, 0.0, 0.0)
    };

    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

    auto timer_callback = [this, width, height, res]() {
      auto msg = generate_map(width, height, res);
      publisher_->publish(msg);
    };

    timer_ = create_wall_timer(std::chrono::seconds(1), timer_callback);
  }

private:
  nav_msgs::msg::OccupancyGrid generate_map(int width, int height, double resolution)
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = "map";

    grid.info.resolution = resolution;
    grid.info.width = width;
    grid.info.height = height;

    // 将地图原点设置为地图中心
    grid.info.origin.position.x = -width * resolution / 2.0;
    grid.info.origin.position.y = -height * resolution / 2.0; 
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.assign(width * height, 0); // 默认空闲

    // 绘制多边形边为占用
    for (size_t i = 0; i < polygon_.size(); i++) {
      auto p1 = polygon_[i];
      auto p2 = polygon_[(i + 1) % polygon_.size()];
      draw_line(p1, p2, grid, resolution);
    }

    return grid;
  }

  void draw_line(const geometry_msgs::msg::Point &p1,
                 const geometry_msgs::msg::Point &p2,
                 nav_msgs::msg::OccupancyGrid &grid,
                 double resolution)
  {
    double origin_x = grid.info.origin.position.x;
    double origin_y = grid.info.origin.position.y;
    
    int x0 = static_cast<int>((p1.x - origin_x) / resolution);
    int y0 = static_cast<int>((p1.y - origin_y) / resolution);
    int x1 = static_cast<int>((p2.x - origin_x) / resolution);
    int y1 = static_cast<int>((p2.y - origin_y) / resolution);

    // Bresenham算法
    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    int x = x0;
    int y = y0;

    while (true) {
      if (x >= 0 && x < (int)grid.info.width && y >= 0 && y < (int)grid.info.height) {
        grid.data[y * grid.info.width + x] = 100;
      }
      if (x == x1 && y == y1) break;
      int e2 = 2 * err;
      if (e2 >= dy) { err += dy; x += sx; }
      if (e2 <= dx) { err += dx; y += sy; }
    }
  }

  std::vector<geometry_msgs::msg::Point> polygon_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PolygonMapNode>());
  rclcpp::shutdown();
  return 0;
}
