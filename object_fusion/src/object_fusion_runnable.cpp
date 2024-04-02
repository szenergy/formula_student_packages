#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "../inc/trajectoryMarkers.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ObjectFusion : public rclcpp::Node
{
public:
    ObjectFusion() : Node("prcp_object_fusion")
    {
        timer = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&ObjectFusion::loop, this));  
        objFusedLeft_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/prcp_obj_list_left_fused", 10);
        objFusedRight_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/prcp_obj_list_right_fused", 10);
        obj_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/pub_proj_coords", 10,  std::bind(&ObjectFusion::obj_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "ObjectFusion has been started");
    }

private:

    std::vector<double> m_leftCones_x; //[m]
    std::vector<double> m_rightCones_x; //[m]
    std::vector<double> m_leftCones_y; //[m]
    std::vector<double> m_rightCones_y; //[m]

    // trajectory marker
    trajectoryMarker point;

    void obj_callback(const std_msgs::msg::Float32MultiArray& input_msg)
    {
        m_leftCones_x.clear();
        m_rightCones_x.clear();
        m_leftCones_y.clear();
        m_rightCones_y.clear();
        // converting published cone positions in local frame
        unsigned long int i = 0;
        while (i < input_msg.data.size())
        {
            if (static_cast<int>(input_msg.data[i]) % 2 == 0)
            {
                // left cone
                m_leftCones_x.push_back(input_msg.data[i+1]);
                m_leftCones_y.push_back(input_msg.data[i+2]);
            }
            else if (static_cast<int>(input_msg.data[i]) % 2 == 1)
            {
                // right cone
                m_rightCones_x.push_back(input_msg.data[i+1]);
                m_rightCones_y.push_back(input_msg.data[i+2]);
            }
            i = i+3;
        }
        printf("number of left cones %ld\n", m_leftCones_x.size());
    }

    void loop()
    {
        auto poseLeft_msg = visualization_msgs::msg::MarkerArray();
        auto poseRight_msg = visualization_msgs::msg::MarkerArray();
        auto poseLeft = geometry_msgs::msg::Pose();
        auto poseRight = geometry_msgs::msg::Pose();

        // Transform from camera frame
        /*tf2::BufferCore tf_buffer;
        geometry_msgs::msg::TransformStamped laser_data_to_other_frame;
        tf2_ros::TransformListener tf2_listener(tf_buffer);

        /*try
        {
            laser_data_to_other_frame = tf_buffer.lookupTransform("base_link", "zed2i_base_link",  tf2::TimePointZero);
            printf("tf found\n");           
        }
        catch(tf2::TransformException &ex)
        {
            printf("transform not found\n");
        }*/

        // assignment of incoming data to the output
        // TODO: real fusion
        for (long unsigned int i=0; i<m_leftCones_x.size(); i++)
        {
            poseLeft.position.x = m_leftCones_x.at(i)/100.0f;
            poseLeft.position.y = m_leftCones_y.at(i)/100.0f;
            point.marker.pose = poseLeft;
            point.marker.id = i;
            poseLeft_msg.markers.push_back(point.marker);
        }
        for (long unsigned int i=0; i<m_rightCones_x.size(); i++)
        {
            poseRight.position.x = m_rightCones_x.at(i)/100.0f;
            poseRight.position.y = m_rightCones_y.at(i)/100.0f;
            point.marker.pose = poseRight;
            point.marker.id = i;
            poseRight_msg.markers.push_back(point.marker);
        }

        // Publish topic
        objFusedLeft_pub->publish(poseLeft_msg);
        objFusedRight_pub->publish(poseRight_msg);
    }
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr objFusedLeft_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr objFusedRight_pub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obj_sub;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectFusion>());
    rclcpp::shutdown();
    return 0;
}