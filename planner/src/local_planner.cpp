#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "../inc/trajectoryMarkers.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class LocalPlanner : public rclcpp::Node
{
public:
    LocalPlanner() : Node("plan_trajectory_planner")
    {
        timer = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&LocalPlanner::loop, this)); 
        traj_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planner/trajectory", 10);
        objFusedLeft_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>("/prcp_obj_list_left_fused", 10,  std::bind(&LocalPlanner::objFusedLeft_callback, this, std::placeholders::_1));
        objFusedRight_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>("/prcp_obj_list_right_fused", 10,  std::bind(&LocalPlanner::objFusedRight_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "LocalPlanner has been started");
    }

private:
    // trajectory marker
    trajectoryMarker point;

    std::vector<double> m_conesLeft_x;
    std::vector<double> m_conesLeft_y;
    std::vector<double> m_conesRight_x;
    std::vector<double> m_conesRight_y;

    void objFusedLeft_callback(const visualization_msgs::msg::MarkerArray input_msg)
    {
        m_conesLeft_x.clear();
        m_conesLeft_y.clear();
        for (unsigned long int i=0; i<input_msg.markers.size(); i++)
        {
            m_conesLeft_x.push_back(input_msg.markers.at(i).pose.position.x);
            m_conesLeft_y.push_back(input_msg.markers.at(i).pose.position.y);
        }
    }

    void loop()
    {
        // Publish pose
        auto pose_msg = visualization_msgs::msg::MarkerArray();
        //updating pose
        auto pose = geometry_msgs::msg::Pose();
        pose.position.x = x;
        pose.position.y = y;
        point.marker.pose = pose;
        point.marker.id = i;

        pose_msg.markers.push_back(point.marker);    
        traj_pub->publish(pose_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_sub_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalPlanner>());
    rclcpp::shutdown();
    return 0;
}