#ifndef MARKER_HPP_
#define MARKER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

void init_debug_marker(visualization_msgs::msg::Marker &debug_marker, double pose_x, double pose_y, int id)
{
    debug_marker.ns = "debug" + std::to_string(id);
    debug_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    debug_marker.action = visualization_msgs::msg::Marker::MODIFY;
    debug_marker.scale.x = 0.4;
    debug_marker.scale.y = 0.4;
    debug_marker.scale.z = 0.4;
    debug_marker.color.r = 0.4;
    debug_marker.color.g = 0.4;
    debug_marker.color.b = 0.8;
    debug_marker.color.a = 1.0;
    debug_marker.id = 3 + id;
    debug_marker.pose.position.x = pose_x;
    debug_marker.pose.position.y = pose_y;
    debug_marker.pose.position.z = 0.0;
    // debug_marker.points.clear();
}

void init_text_debug_marker(visualization_msgs::msg::Marker &debug_marker)
{
    debug_marker.ns = "debug_text";
    debug_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    debug_marker.action = visualization_msgs::msg::Marker::MODIFY;
    debug_marker.scale.z = 0.4;
    debug_marker.color.r = 0.9;
    debug_marker.color.g = 0.9;
    debug_marker.color.b = 0.9;
    debug_marker.color.a = 1.0;
    debug_marker.id = 4;
    debug_marker.pose.position.x = 0.0;
    debug_marker.pose.position.y = 0.0;
    debug_marker.pose.position.z = 0.0;
    // debug_marker.points.clear();
}

void init_center_marker(visualization_msgs::msg::Marker &debug_marker, double pose_x, double pose_y, int id)
{
    // debug_marker.ns = "cluster" + std::to_string(id);
    debug_marker.ns = "cluster_center";
    debug_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    debug_marker.action = visualization_msgs::msg::Marker::MODIFY;
    debug_marker.scale.x = 0.8;
    debug_marker.scale.y = 0.8;
    debug_marker.scale.z = 0.8;
    debug_marker.color.r = 0.8;
    debug_marker.color.g = 0.8;
    debug_marker.color.b = 0.4;
    debug_marker.color.a = 1.0;
    debug_marker.id = id;
    debug_marker.pose.position.x = pose_x;
    debug_marker.pose.position.y = pose_y;
    debug_marker.pose.position.z = 0.0;
    // debug_marker.points.clear();
}

void init_line_marker(visualization_msgs::msg::Marker &line1_marker, int id)
{
    //visualization_msgs::msg::Marker line1_marker;
    line1_marker.ns = "line" + std::to_string(id);
    line1_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    line1_marker.action = visualization_msgs::msg::Marker::MODIFY;
    line1_marker.scale.x = 0.4;
    line1_marker.color.r = 0.2;
    line1_marker.color.g = 0.6;
    line1_marker.color.b = 0.8;
    line1_marker.color.a = 1.0;
    line1_marker.id = id;
    line1_marker.pose.position.x = 0.0;
    line1_marker.pose.position.y = 0.0;
    line1_marker.pose.position.z = 0.0;
    line1_marker.points.clear();
}

void add_point_to_line_marker(visualization_msgs::msg::Marker &line1_marker, double x, double y)
{
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    line1_marker.points.push_back(p);
}
 

#endif // MARKER_HPP_