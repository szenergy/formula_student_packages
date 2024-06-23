#include <chrono>
#include <functional>
#include <memory>
#include <string>
// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

using namespace std::chrono_literals;

class FliterVehicle : public rclcpp::Node
{
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &param : parameters)
        {
            // RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
            if (param.get_name() == "cloud_in_topic")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "cloud_in_topic: " << param.get_value<std::string>());
            }
            else if (param.get_name() == "verbose1")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "verbose1: " << param.get_value<bool>());
                verbose1 = param.get_value<bool>();
            }
            else if (param.get_name() == "verbose2")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "verbose2: " << param.get_value<bool>());
                verbose2 = param.get_value<bool>();
            }
            else if (param.get_name() == "minX_over")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "minX_over: " << param.get_value<float>());
                minX_over = param.get_value<float>();
            }
            else if (param.get_name() == "minY_over")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "minY_over: " << param.get_value<float>());
                minY_over = param.get_value<float>();
            }
            else if (param.get_name() == "minZ_over")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "minZ_over: " << param.get_value<float>());
                minZ_over = param.get_value<float>();
            }
            else if (param.get_name() == "maxX_over")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "maxX_over: " << param.get_value<float>());
                maxX_over = param.get_value<float>();
            }
            else if (param.get_name() == "maxY_over")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "maxY_over: " << param.get_value<float>());
                maxY_over = param.get_value<float>();
            }
            else if (param.get_name() == "maxZ_over")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "maxZ_over: " << param.get_value<float>());
                maxZ_over = param.get_value<float>();
            }
            else if (param.get_name() == "minX_vehicle")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "minX_vehicle: " << param.get_value<float>());
                minX_vehicle = param.get_value<float>();
            }
            else if (param.get_name() == "minY_vehicle")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "minY_vehicle: " << param.get_value<float>());
                minY_vehicle = param.get_value<float>();
            }
            else if (param.get_name() == "maxX_vehicle")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "maxX_vehicle: " << param.get_value<float>());
                maxX_vehicle = param.get_value<float>();
            }
            else if (param.get_name() == "maxY_vehicle")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "maxY_vehicle: " << param.get_value<float>());
                maxY_vehicle = param.get_value<float>();
            }
            
            else if (param.get_name() == "crop_box_array")
            {
                std::string tempstr = "crop_box_array...";
                crop_box_array = param.get_value<std::vector<double>>();
                //todo: nice info msg
                RCLCPP_INFO_STREAM(this->get_logger(), tempstr);
            }
            else
            {
                result.successful = false;
                result.reason = "failed";
            }
        }
        return result;
    }

public:
    FliterVehicle() : Node("filter_vehicle"), count_(0)
    {
        // parameters
        this->declare_parameter<std::string>("cloud_in_topic", "input_cloud");
        this->declare_parameter<bool>("verbose1", false);
        this->declare_parameter<bool>("verbose2", true);
        this->declare_parameter<float>("minX_over", -200.0);
        this->declare_parameter<float>("minY_over", -25.0);
        this->declare_parameter<float>("minZ_over", -2.0);
        this->declare_parameter<float>("maxX_over", 200.0);
        this->declare_parameter<float>("maxY_over", 25.0);
        this->declare_parameter<float>("maxZ_over", -0.01);
        this->declare_parameter<float>("minX_vehicle", -5.0);
        this->declare_parameter<float>("minY_vehicle", -5.0);
        this->declare_parameter<float>("maxX_vehicle", 5.0);
        this->declare_parameter<float>("maxY_vehicle", 5.0);
        this->declare_parameter<std::vector<double>>("crop_box_array",
            std::vector<double>{-1.2, 1.2, -1.2, 1.2, -1.2, 1.2});

        this->get_parameter("cloud_in_topic", cloud_in_topic);
        this->get_parameter("verbose1", verbose1);
        this->get_parameter("verbose2", verbose2);
        this->get_parameter("minX_over", minX_over);
        this->get_parameter("minY_over", minY_over);
        this->get_parameter("minZ_over", minZ_over);
        this->get_parameter("maxX_over", maxX_over);
        this->get_parameter("maxY_over", maxY_over);
        this->get_parameter("maxZ_over", maxZ_over);
        this->get_parameter("minX_vehicle", minX_vehicle);
        this->get_parameter("minY_vehicle", minY_vehicle);
        this->get_parameter("maxX_vehicle", maxX_vehicle);
        this->get_parameter("maxY_vehicle", maxY_vehicle);
        this->get_parameter("crop_box_array", crop_box_array);


        pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_filter_output", 10);
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_in_topic, rclcpp::SensorDataQoS().keep_last(1), std::bind(&FliterVehicle::lidar_callback, this, std::placeholders::_1));
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&FliterVehicle::parametersCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "FliterVehicle node has been started.");
        RCLCPP_INFO_STREAM(this->get_logger(), "cloud_in_topic: " << this->get_parameter("cloud_in_topic").as_string());

    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
    {
        // RCLCPP_INFO(this->get_logger(), "frame_id: '%s'", input_msg->header.frame_id.c_str());

        // Filter point cloud data
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input_msg, *cloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_over(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_vehicle(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::CropBox<pcl::PointXYZI> crop_over, crop_vehicle;   //todo: use or clean up?
        std::vector<pcl::CropBox<pcl::PointXYZI>*> crop_boxes;  //todo: single variable OR implement parallelism

        //remnant? --->
        crop_over.setInputCloud(cloud);
        // Filter out points outside of the box
        crop_over.setMin(Eigen::Vector4f(minX_over, minY_over, minZ_over, 1.0));
        crop_over.setMax(Eigen::Vector4f(maxX_over, maxY_over, maxZ_over, 1.0));
        crop_over.filter(*cloud_over);
        
        crop_vehicle.setInputCloud(cloud_over);
        // Filter out points inside of the box -- Negative
        crop_vehicle.setMin(Eigen::Vector4f(minX_vehicle, minY_vehicle, minZ_over, 1.0));
        crop_vehicle.setMax(Eigen::Vector4f(maxX_vehicle, maxY_vehicle, maxZ_over, 1.0));
        crop_vehicle.setNegative(true);
        crop_vehicle.filter(*cloud_vehicle);
        // <--- remnant?

        // Array-based crop box! (multiple boxes, sequential filtering)
        std::vector<float> passvec; //temp - todo: proper conversion instead?
        passvec.resize(crop_box_array.size());
        for (int i=crop_box_array.size()-1; i>=0; i--) passvec[i] = crop_box_array[i];
        if (!(crop_box_array.size() % 6))   //sanity check
        {
            crop_boxes.resize(crop_box_array.size() / 6);
            std::cerr << crop_box_array[0];
            int s = crop_box_array.size() / 6;
            RCLCPP_INFO_ONCE(this->get_logger(),
                "BoxFilter parameters:\n#\t minX\t maxX\t minY\t maxY\t minZ\t maxZ\n");
            int j;
            for (int i = 0; i < s; i++) //per crop box
            {
                j = i*6;
                //RCLCPP_INFO(this->get_logger(),       //debug: swap w/ line below
                RCLCPP_INFO_ONCE(this->get_logger(),    //todo: once PER ROW!
                    "\n%d\t%+7.2f\t%+7.2f\t%+7.2f\t%+7.2f\t%+7.2f\t%+7.2f\n", i,
                    passvec[j],     //minX
                    passvec[j+1],   //maxX
                    passvec[j+2],   //minY
                    passvec[j+3],   //maxY
                    passvec[j+4],   //minZ
                    passvec[j+5]);  //maxZ
                crop_boxes[i] = new pcl::CropBox<pcl::PointXYZI>;
                if (!i) crop_boxes[i]->setInputCloud(cloud);        //first time (original cloud)
                else crop_boxes[i]->setInputCloud(cloud_cropped);   //repeat on previous result
                crop_boxes[i]->setMin(Eigen::Vector4f(passvec[j], passvec[j+2], passvec[j+4], 1.0));
                crop_boxes[i]->setMax(Eigen::Vector4f(passvec[j+1], passvec[j+3], passvec[j+5], 1.0));
                crop_boxes[i]->setNegative(true);
                crop_boxes[i]->filter(*cloud_cropped);
            }
            RCLCPP_INFO_ONCE(this->get_logger(), "---");    //todo: proper layout (one msg, newline per row)
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Incorrect number of Crop Box parameters! \
                (Should be a multiple of 6: minX, maxX, minY, maxY, minZ, maxZ)");  //todo: test
        }

        // Convert to ROS data type
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_cropped, output_msg);
        // Add the same frame_id as th input, it is not included in pcl PointXYZI
        output_msg.header.frame_id = input_msg->header.frame_id;
        // Publish the data as a ROS message
        pub_lidar_->publish(output_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    std::string cloud_in_topic = "pointcloud_topic";
    float minX_over = -10.0, minY_over = -5.0, minZ_over = -2.0;
    float maxX_over = +10.0, maxY_over = +5.0, maxZ_over = -0.15;
    float minX_vehicle = -5.0, minY_vehicle = -5.0;
    float maxX_vehicle = +5.0, maxY_vehicle = +5.0;
    std::vector<double> crop_box_array = {-0.8, 0.8, -0.8, 0.8, -0.8, 0.8};
    bool verbose1 = false;
    bool verbose2 = true;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FliterVehicle>());
    rclcpp::shutdown();
    return 0;
}