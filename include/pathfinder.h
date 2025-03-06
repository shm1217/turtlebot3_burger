#pragma once

#include "rclcpp/rclcpp.hpp"
#include <map>
#include <set>
#include <vector>

struct PathNode
{
    double x, y;
    double f, g, h;

    PathNode(double r = -1, double c = -1);

    bool operator>(const PathNode &other) const;
    bool operator==(const PathNode &other) const;
};

std::vector<PathNode> FindPath(const std::vector<std::vector<int>> &graph, const PathNode &start, const PathNode &goal);
std::vector<std::vector<int>> createGrid(double width, double height, const std::set<std::pair<int, int>> &obstacles);