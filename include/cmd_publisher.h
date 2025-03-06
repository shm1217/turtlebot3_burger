#ifndef ROS2_TERM_PROJECT_CMD_PUBLISHER_H
#define ROS2_TERM_PROJECT_CMD_PUBLISHER_H

#include "geometry_msgs/msg/twist.hpp"
#include "octomap/OcTree.h"
#include "pathfinder.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include <queue>
#include <set>
#include <vector>
#pragma once
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/publisher.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <dynamicEDT3D/dynamicEDTOctomap.h>

class CmdPublisher : public rclcpp::Node
{
public:
    CmdPublisher();

private:
    void timer_tf_callback();
    void timer_cmd_callback();
    void moverobot();
    void visualizePath(const std::vector<PathNode> &path);
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    double x, y, z;
    double yaw;
    double goal_x = 0.0;
    double goal_y = 0.0;
    bool position_updated = false;
    bool obstacle_detected = false;
    int current_waypoint = 0;
    bool goal_received;

    double Kp_linear = 0.8;  // 선속도 P 게인 1.0
    double Kd_linear = 0.4;  // 선속도 D 게인 0.5
    double Kp_angular = 0.4; // 회전 P 게인 1.0
    double Kd_angular = 0.1; // 회전 D 게인 0.3
    rclcpp::Time prev_time;
    
    int map_width;      
    int map_height;     
    double resolution; 
    double origin_x;    
    double origin_y;
    bool map_update=false;

    rclcpp::TimerBase::SharedPtr timer_cmd, timer_tf;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
    //rclcpp::Subscription<OctomapMsg>::SharedPtr sub_octomap;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker marker;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map;
};

#endif // ROS2_TERM_PROJECT_CMD_PUBLISHER_H
