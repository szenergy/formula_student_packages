#ifndef ROS_HANDLER_H
#define ROS_HANDLER_H

#include "ProjectionHandler.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <vector>

class RosHandler : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_coords;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_coords;

    OnSetParametersCallbackHandle::SharedPtr cb_handle_;
public:
    RosHandler();
    void float32_callback(const std_msgs::msg::Float32MultiArray& msg);
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
};

#endif