#ifndef ROS_HANDLER_H
#define ROS_HANDLER_H

#include "ProjectionHandler.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class RosHandler : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_coords;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_coords;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
public:
    RosHandler();
    void float32_callback(const std_msgs::msg::Float32MultiArray& msg);
    // void paramsCallback(my_dyn_rec::ValidateConfig &config, uint32_t level);
};

#endif