
#include "cmd_publisher.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <set>
#include <sstream>
#include <thread>
#include <tuple>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;
std::set<std::pair<double, double>> green_obstacles;
std::set<std::pair<int, int>> grid_obstacles;
double cell_size = 0.1;

CmdPublisher::CmdPublisher() : Node("cmd_publisher"), goal_x(0.0), goal_y(0.0), goal_received(false)
{
    // Publisher
    pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Subscribers
    //sub_octomap = this->create_subscription<OctomapMsg>(
        //"octomap_full", 10, std::bind(&CmdPublisher::octomap_callback, this, _1));
    sub_map = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&CmdPublisher::map_callback, this, std::placeholders::_1));

    // TF listener
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Timers
    timer_tf = this->create_wall_timer(
        50ms, std::bind(&CmdPublisher::timer_tf_callback, this));
    timer_cmd = this->create_wall_timer(
        500ms, std::bind(&CmdPublisher::timer_cmd_callback, this));

    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot1/marker", 10);
    marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("visualization/marker", 10);
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "move_base_simple/goal", 10,
        std::bind(&CmdPublisher::goal_callback, this, std::placeholders::_1));
    prev_time = this->now();
    //last_replan_time = this->now();
}
void CmdPublisher::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;
    goal_received = true;
    RCLCPP_INFO(this->get_logger(), "Received goal: x = %f, y = %f", goal_x, goal_y);
}
void CmdPublisher::timer_tf_callback()
{
    geometry_msgs::msg::TransformStamped t;
    try
    {
        t = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Could not transform map to base_scan");
        return;
    }

    x = t.transform.translation.x;
    y = t.transform.translation.y;
    z = t.transform.translation.z;

    tf2::Quaternion q(
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);

    position_updated = true;
}
void CmdPublisher::timer_cmd_callback()
{
    if ( not goal_received or not position_updated)
    {
        return;
    }
          
    moverobot();
}
// 이전 오차 저장 변수
static double prev_linear_error = 0.0;
static double prev_angular_error = 0.0; // 이전 시간

void CmdPublisher::moverobot()
{
    if ( not position_updated)
    {
        return;
    }

    int start_x_idx = static_cast<int>(x-origin_x / cell_size);
    int start_y_idx = static_cast<int>(y-origin_y / cell_size);
    int goal_x_idx = static_cast<int>(goal_x-origin_x/ cell_size);
    int goal_y_idx = static_cast<int>(goal_y-origin_y/ cell_size);
    std::vector<std::vector<int>> grid = createGrid(map_width, map_height, grid_obstacles);
    std::vector<PathNode> path = FindPath(grid, PathNode(start_x_idx, start_y_idx), PathNode(goal_x_idx, goal_y_idx));

    if (path.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No valid path found!");
        return;
    }

    visualizePath(path);

    double lookahead_distance = 0.3; // Pure Pursuit에서 사용할 lookahead 거리

    // 가장 먼 유효한 waypoint 찾기
    int target_waypoint = current_waypoint;
    if (current_waypoint < path.size())
    {
        double waypoint_x = (path[current_waypoint].x + 0.5) * cell_size;
        double waypoint_y = (path[current_waypoint].y + 0.5) * cell_size;
        double distance_to_waypoint = sqrt(pow(waypoint_x - x, 2) + pow(waypoint_y - y, 2));

        if (distance_to_waypoint > lookahead_distance)
        {
            return;
        }
        target_waypoint = current_waypoint;
    }

    if (target_waypoint < path.size())
    {
        double next_x = (path[target_waypoint].x + 0.5) * cell_size;
        double next_y = (path[target_waypoint].y + 0.5) * cell_size;

        rclcpp::Time current_time = this->now();
        double dt = (current_time - prev_time).seconds();

        double linear_error = sqrt(pow(next_x - x, 2) + pow(next_y - y, 2));
        double linear_derivative = (linear_error - prev_linear_error) / dt;
        double linear_velocity = Kp_linear * linear_error + Kd_linear * linear_derivative;

        double angle_to_goal = atan2(next_y - y, next_x - x);
        double angular_error = angle_to_goal - yaw;
        while (angular_error > M_PI)
            angular_error -= 2 * M_PI;
        while (angular_error < -M_PI)
            angular_error += 2 * M_PI;

        double angular_derivative = (angular_error - prev_angular_error) / dt;
        double angular_velocity = Kp_angular * angular_error + Kd_angular * angular_derivative;

        prev_linear_error = linear_error;
        prev_angular_error = angular_error;
        prev_time = current_time;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::max(0.1, std::min(0.3, linear_velocity));
        cmd.angular.z = std::max(-2.0, std::min(2.0, angular_velocity));

        pub_cmd->publish(cmd);

        RCLCPP_INFO(this->get_logger(), "target_waypoint=%d, path.size=%ld", target_waypoint, path.size());
        RCLCPP_INFO(this->get_logger(), "현재 위치: (%f, %f), 목표: (%f, %f)", x, y, next_x, next_y);
        RCLCPP_INFO(this->get_logger(), "속도: linear=%f, angular=%f", cmd.linear.x, cmd.angular.z);

        if (linear_error < 0.1)
        {
            current_waypoint = target_waypoint + 1;
        }
    }

    if (abs(x - goal_x) + abs(y - goal_y) < 0.2 || current_waypoint >= path.size())
    {
        RCLCPP_INFO(this->get_logger(), "목적지 도착");
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = 0.0;
        cmd.linear.x = 0.0;
        pub_cmd->publish(cmd);
        goal_received = false;
        return;
    }
}
void CmdPublisher::visualizePath(const std::vector<PathNode> &path)
{
    // 경로를 RViz에 마커로 표시하기 위한 마커 메시지 생성
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map"; // 경로가 그려질 프레임 설정
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "path_visualization";                          // 네임스페이스
    marker.id = 0;                                             // 마커 ID (여러 마커를 사용할 때는 유니크한 ID가 필요)
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // 경로를 선으로 표시
    marker.action = visualization_msgs::msg::Marker::ADD;      // 마커를 추가하는 동작

    marker.scale.x = 0.1;  // 선 두께
    marker.color.r = 1.0f; // 빨간색
    marker.color.g = 0.0f; // 초록색
    marker.color.b = 0.0f; // 파란색
    marker.color.a = 1.0f; // 투명도

    // 경로 노드들을 마커 포인트로 추가
    for (const auto &node : path)
    {
        geometry_msgs::msg::Point p;
        p.x = (node.x + 0.5) * cell_size; // 셀 중심 좌표 변환
        p.y = (node.y + 0.5) * cell_size;
        p.z = 0.0; // Z 좌표는 0으로 설정 (평면에서의 경로)
        marker.points.push_back(p);
    }
    marker_pub->publish(marker);
}

void CmdPublisher::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_width = msg->info.width;
    map_height = msg->info.height;
    resolution = msg->info.resolution;
    origin_x = msg->info.origin.position.x;
    origin_y = msg->info.origin.position.y;
    int grid_width = static_cast<int>(map_width / cell_size);
    int grid_height = static_cast<int>(map_height / cell_size);

    grid_obstacles.clear();
    
    for (int y = 0; y < grid_height; y++)
    {
        for (int x = 0; x < grid_width; x++)
        {
            int index = y * map_width + x;
            if (msg->data[index] > 50) // 점유 값이 50 이상이면 장애물로 판단
            {
                int grid_x = x;
                int grid_y = y;
                grid_obstacles.insert({grid_x, grid_y});
            }
        }
    }
}


