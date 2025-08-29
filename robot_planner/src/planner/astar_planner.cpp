#include "robot_planner/planner/astar_planner.hpp"

namespace robot_planner
{

nav_msgs::msg::Path AStar_Planner::aStarSearch(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const nav_msgs::msg::OccupancyGrid & costmap)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = costmap.header.frame_id;
    // path.header.stamp = rclcpp::Clock().now(); // 如果不想依赖 rclcpp，可以注释掉

    unsigned int nx = costmap.info.width;
    unsigned int ny = costmap.info.height;

    // 起点终点换算成 map index
    auto worldToMap = [&](double wx, double wy, unsigned int &mx, unsigned int &my) -> bool {
        if (wx < costmap.info.origin.position.x || wy < costmap.info.origin.position.y) return false;

        mx = static_cast<unsigned int>((wx - costmap.info.origin.position.x) / costmap.info.resolution);
        my = static_cast<unsigned int>((wy - costmap.info.origin.position.y) / costmap.info.resolution);

        return (mx < nx && my < ny);
    };

    unsigned int start_x, start_y, goal_x, goal_y;
    if (!worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
        !worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
        return path; // 空路径
    }

    // OpenSet
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
    std::unordered_set<unsigned int> closedSet;
    std::unordered_map<unsigned int, double> g_values;

    // heuristic
    auto heuristic = [&](unsigned int x, unsigned int y) {
        return std::hypot(static_cast<int>(x) - static_cast<int>(goal_x),
                          static_cast<int>(y) - static_cast<int>(goal_y));
    };

    openSet.emplace(start_x, start_y, 0.0, heuristic(start_x, start_y));
    g_values[toIndex(start_x, start_y, nx)] = 0.0;

    const int dx[8]   = {-1,-1,-1, 0,0, 1,1,1};
    const int dy[8]   = {-1, 0, 1,-1,1,-1,0,1};
    const double cost[8] = {M_SQRT2,1.0,M_SQRT2,1.0,1.0,M_SQRT2,1.0,M_SQRT2};

    NodePtr finalNode = nullptr;

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        unsigned int currentKey = toIndex(current.x, current.y, nx);
        if (closedSet.count(currentKey)) continue;
        closedSet.insert(currentKey);

        // 到达目标
        if (current.x == goal_x && current.y == goal_y) {
            finalNode = std::make_shared<Node>(current);
            break;
        }

        // 遍历邻居
        for (int i = 0; i < 8; ++i) {
            int nx_ = static_cast<int>(current.x) + dx[i];
            int ny_ = static_cast<int>(current.y) + dy[i];

            if (nx_ < 0 || ny_ < 0 || nx_ >= static_cast<int>(nx) || ny_ >= static_cast<int>(ny)) {
                continue;
            }

            unsigned int neighborKey = toIndex(nx_, ny_, nx);

            // 检查是否障碍物
            if (costmap.data[neighborKey] > 50) { // 阈值可调
                continue;
            }

            double new_g = current.g + cost[i];

            if (g_values.find(neighborKey) == g_values.end() || new_g < g_values[neighborKey]) {
                g_values[neighborKey] = new_g;
                double new_h = heuristic(nx_, ny_);
                openSet.emplace(nx_, ny_, new_g, new_h, std::make_shared<Node>(current));
            }
        }
    }

    // 生成路径
    if (finalNode) {
        NodePtr node = finalNode;
        while (node) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = costmap.info.origin.position.x + (node->x + 0.5) * costmap.info.resolution;
            pose.pose.position.y = costmap.info.origin.position.y + (node->y + 0.5) * costmap.info.resolution;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);

            node = node->parent;
        }
        std::reverse(path.poses.begin(), path.poses.end());
    }

    return path;
}


} // namespace planner