#include "robot_planner/planner/astar_planner.hpp"

namespace robot_planner
{
// A*算法
std::vector<std::pair<unsigned int, unsigned int>> AStar_Planner::aStarSearch(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const nav_msgs::msg::OccupancyGrid & costmap)
{
    std::vector<std::pair<unsigned int, unsigned int>> path;

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
            path.emplace_back(node->x, node->y);
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
    }

    return path;
}

// 数组转换为path
nav_msgs::msg::Path AStar_Planner::toPathMsg(
    const std::vector<std::pair<unsigned int, unsigned int>> & grid_path,
    const nav_msgs::msg::OccupancyGrid & costmap)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = costmap.header.frame_id;

    for (auto &p : grid_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = costmap.info.origin.position.x + (p.first + 0.5) * costmap.info.resolution;
        pose.pose.position.y = costmap.info.origin.position.y + (p.second + 0.5) * costmap.info.resolution;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    return path;
}

// 计算距离场EDF
std::vector<std::vector<double>> AStar_Planner::computeDistanceField(
    const nav_msgs::msg::OccupancyGrid & costmap)
{
    int obstacle_threshold = 50;

    unsigned int width  = costmap.info.width;
    unsigned int height = costmap.info.height;
    double res = costmap.info.resolution;

    // 初始化
    std::vector<std::vector<double>> dist(height, std::vector<double>(width, 1e9));
    std::queue<std::pair<int,int>> q;

    // 所有障碍物先入队
    for (unsigned int y = 0; y < height; y++) {
        for (unsigned int x = 0; x < width; x++) {
            unsigned int idx = y * width + x;
            if (costmap.data[idx] >= obstacle_threshold) {  // 注意用 >=
                dist[y][x] = 0.0;
                q.push({x,y});
            }
        }
    }

    // 8 邻域偏移
    const std::vector<std::pair<int,int>> directions = {
        {1,0}, {-1,0}, {0,1}, {0,-1},
        {1,1}, {-1,-1}, {1,-1}, {-1,1}
    };

    // BFS
    while (!q.empty()) {
        auto [cx, cy] = q.front();
        q.pop();

        for (auto [dx,dy] : directions) {
            int nx = cx + dx;
            int ny = cy + dy;
            if (nx < 0 || ny < 0 || nx >= (int)width || ny >= (int)height) continue;

            double step = (dx == 0 || dy == 0) ? res : res * M_SQRT2;
            double new_dist = dist[cy][cx] + step;

            if (new_dist < dist[ny][nx]) {
                dist[ny][nx] = new_dist;
                q.push({nx,ny});
            }
        }
    }

    return dist;
}

// 梯度下降法平滑路径
std::vector<std::pair<double, double>> AStar_Planner::smoothPathWithGradientDescent(
    const std::vector<std::pair<unsigned int, unsigned int>> &grid_path,
    const std::vector<std::vector<double>> &dist_field,
    const nav_msgs::msg::OccupancyGrid &costmap)
{
    int iterations = 100;
    double alpha = 0.2;   // 平滑权重
    double beta = 0.1;    // 障碍物权重
    double step_size = 0.1;

    unsigned int width  = costmap.info.width;
    unsigned int height = costmap.info.height;
    double res = costmap.info.resolution;

    // 1. 把路径从grid坐标转到世界坐标 (double)
    std::vector<std::pair<double, double>> path;

    for (auto &p : grid_path) {
        double wx = costmap.info.origin.position.x + (p.first + 0.5) * res;
        double wy = costmap.info.origin.position.y + (p.second + 0.5) * res;
        path.emplace_back(wx, wy);
    }

    // 2. 梯度下降优化
    for (int it = 0; it < iterations; ++it) {
        for (size_t i = 1; i + 1 < path.size(); i++) { // 保留起点终点不动
            double x = path[i].first;
            double y = path[i].second;

            // ---- (1) 平滑梯度 (和前后点连线靠拢) ----
            double dx_smooth = (path[i-1].first + path[i+1].first - 2*x);
            double dy_smooth = (path[i-1].second + path[i+1].second - 2*y);

            // ---- (2) 障碍物梯度 (距离场梯度下降) ----
            // 转到grid坐标
            int gx = (int)((x - costmap.info.origin.position.x) / res);
            int gy = (int)((y - costmap.info.origin.position.y) / res);

            double dist = dist_field[gy][gx];
            double d_thresh = 0.1; // 阈值，单位与地图一致（米）

            double dx_obs = 0.0, dy_obs = 0.0;
            // 靠近障碍物时才推开
            if (dist < d_thresh && gx > 0 && gx+1 < (int)width && gy > 0 && gy+1 < (int)height) {

                // 计算数值梯度 ∇dist
                double grad_x = (dist_field[gy][gx+1] - dist_field[gy][gx-1]) / (2*res);
                double grad_y = (dist_field[gy+1][gx] - dist_field[gy-1][gx]) / (2*res);
                // 想要远离障碍物，所以梯度要取正
                dx_obs = grad_x;
                dy_obs = grad_y;
            }

            // ---- (3) 合成梯度 ----
            double dx = alpha * dx_smooth + beta * dx_obs;
            double dy = alpha * dy_smooth + beta * dy_obs;

            // ---- (4) 更新点位置 ----
            path[i].first  += step_size * dx;
            path[i].second += step_size * dy;
        }
    }

    return path;
}

} // namespace planner