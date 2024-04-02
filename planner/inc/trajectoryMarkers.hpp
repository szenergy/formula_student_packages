#include "visualization_msgs/msg/marker_array.hpp"

struct trajectoryMarker
    {
        trajectoryMarker(): marker()
            {
                marker.header.frame_id = "map";
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.scale.x = 0.35;
                marker.scale.y = 0.35;
                marker.scale.z = 0.35;
                marker.id = 0;
                marker.lifetime.nanosec = 2e8;
                marker.color.r = 0.85;
                marker.color.g = 0.03;
                marker.color.b = 0.89;
                marker.color.a = 1.0;
            }
        visualization_msgs::msg::Marker marker;
    };
