#pragma once

#include <memory>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <vector>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot_planner
{

class AStar_Planner
{
public:
    nav_msgs::msg::Path aStarSearch(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal,
        const nav_msgs::msg::OccupancyGrid & costmap);

private:
    struct Node;
    using NodePtr = std::shared_ptr<Node>;

    struct Node {
        unsigned int x, y;
        double f, g, h;
        NodePtr parent;

        Node(unsigned int x, unsigned int y, double g, double h, NodePtr parent = nullptr)
            : x(x), y(y), g(g), h(h), f(g + h), parent(parent) {}

        bool operator>(const Node& other) const {
            return f > other.f;
        }
    };

    inline unsigned int toIndex(unsigned int x, unsigned int y, unsigned int width) {
        return y * width + x;
    }
};

} // namespace robot_planner