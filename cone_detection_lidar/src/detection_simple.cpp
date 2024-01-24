#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class DetectionSimple : public rclcpp::Node
{
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &param : parameters)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
            if (param.get_name() == "cloud_in_topic")
            {
                cloud_in_topic = param.as_string();
                // sub_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_in_topic, 10, std::bind(&PointCloudToGrid::lidar_callback, this, std::placeholders::_1));
            }
            if (param.get_name() == "verbose1")
            {
                verbose1 = param.as_bool();
            }
            if (param.get_name() == "verbose2")
            {
                verbose2 = param.as_bool();
            }
        }
        return result;
    }

public:
    DetectionSimple() : Node("detection_simple"), count_(0)
    {
        this->declare_parameter<std::string>("cloud_in_topic", cloud_in_topic);
        this->declare_parameter<bool>("verbose1", verbose1);
        this->declare_parameter<bool>("verbose2", verbose2);

        this->get_parameter("cloud_in_topic", cloud_in_topic);
        this->get_parameter("verbose1", verbose1);
        this->get_parameter("verbose2", verbose2);

        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&DetectionSimple::timer_callback, this));
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DetectionSimple::parametersCallback, this, std::placeholders::_1));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    std::string cloud_in_topic = "nonground";
    bool verbose1 = true, verbose2 = false;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionSimple>());
    rclcpp::shutdown();
    return 0;
}