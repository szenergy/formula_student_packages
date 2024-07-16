#include <chrono>
#include <functional>
#include <memory>
#include <string>
// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
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
            if (param.get_name() == "cam_cones_topic")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "cam_cones_topic: " << param.get_value<std::string>());
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
            else if (param.get_name() == "toggle_box_filter")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "toggle_box_filter: " << param.get_value<bool>());
                toggle_box_filter = param.get_value<bool>();
            }
            else if (param.get_name() == "toggle_cam_filter")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "toggle_cam_filter: " << param.get_value<bool>());
                toggle_cam_filter = param.get_value<bool>();
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
                crop_box_array = param.get_value<std::vector<double>>();
                printcfg();
            }
            else if (param.get_name() == "cam_cone_radius")
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "cam_cone_radius: " << param.get_value<float>());
                cam_cone_radius = param.get_value<float>();
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
        this->declare_parameter<std::string>("cam_cones_topic", "input_cones");
        this->declare_parameter<bool>("verbose1", false);
        this->declare_parameter<bool>("verbose2", true);
        this->declare_parameter<bool>("toggle_box_filter", true);
        this->declare_parameter<bool>("toggle_cam_filter", false);
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
        this->declare_parameter<float>("cam_cone_radius", 0.5);

        this->get_parameter("cloud_in_topic", cloud_in_topic);
        this->get_parameter("cam_cones_topic", cam_cones_topic);
        this->get_parameter("verbose1", verbose1);
        this->get_parameter("verbose2", verbose2);
        this->get_parameter("toggle_box_filter", toggle_box_filter);
        this->get_parameter("toggle_cam_filter", toggle_cam_filter);
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
        this->get_parameter("cam_cone_radius", cam_cone_radius);
/*
        rclcpp::TimerBase::SharedPtr timer =
            this->create_wall_timer(50ms, std::bind(&FliterVehicle::timer_callback, this));
        for (float i = -15; i < 0; i += 0.25)
            for (float j = -7; j < 7; j += 0.25)
                for (float k = -1.5; k < 1.5; k += 0.5)
                    test_cloud.points.push_back(pcl::PointXYZI(i,j,k,1.0));
*/

        pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_filter_output", 10);
        //pub_testcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("testcloud", 10);
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_in_topic, rclcpp::SensorDataQoS().keep_last(1), std::bind(&FliterVehicle::lidar_callback, this, std::placeholders::_1));
        sub_cam_cones_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(cam_cones_topic, rclcpp::SensorDataQoS().keep_last(1), std::bind(&FliterVehicle::cam_cones_callback, this, std::placeholders::_1));
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&FliterVehicle::parametersCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "FliterVehicle node has been started.");
        RCLCPP_INFO_STREAM(this->get_logger(), "cloud_in_topic: " << this->get_parameter("cloud_in_topic").as_string());
        RCLCPP_INFO_STREAM(this->get_logger(), "cam_cones_topic: " << this->get_parameter("cam_cones_topic").as_string());
        printcfg();

    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "First PCL input recieved, BoxFilter is turned %s",
            (toggle_box_filter ? "ON" : "OFF"));
        if (!toggle_box_filter) return;
        // RCLCPP_INFO(this->get_logger(), "frame_id: '%s'", input_msg->header.frame_id.c_str());

        // Filter point cloud data
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input_msg, *cloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_over(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_vehicle(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cones(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud = cloud_cropped;
        pcl::CropBox<pcl::PointXYZI> crop_over, crop_vehicle;   //todo: use or clean up?
        pcl::CropBox<pcl::PointXYZI>* crop_box;

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
            int s = crop_box_array.size() / 6;
            int j;
            for (int i = 0; i < s; i++) //per crop box
            {
                j = i*6;
                crop_box = new pcl::CropBox<pcl::PointXYZI>;
                if (!i) crop_box->setInputCloud(cloud);        //first time (original cloud)
                else crop_box->setInputCloud(cloud_cropped);   //repeat on previous result
                crop_box->setMin(Eigen::Vector4f(passvec[j], passvec[j+2], passvec[j+4], 1.0));
                crop_box->setMax(Eigen::Vector4f(passvec[j+1], passvec[j+3], passvec[j+5], 1.0));
                crop_box->setNegative(true);
                crop_box->filter(*cloud_cropped);
                delete crop_box;
            }
        }
        else RCLCPP_ERROR(this->get_logger(), ERROR_TEXT_PARAM_NUM);

        if (toggle_cam_filter)
        {
            int s = cloud_cropped->points.size();
            int c = cones.size();
            float x, y, r,
                rs = cam_cone_radius * cam_cone_radius;
            for (int i = 0; i < s; i++)     //for every point
            {
                for (int j = 0; j < c; j++) //for every cone
                {
                    x = cones[j].x - cloud_cropped->points[i].x;
                    y = cones[j].y - cloud_cropped->points[i].y;
                    r = x * x + y * y;  //squared distance
                    if (r < rs)         //no need for sqrt() for comparison
                    {
                        //RCLCPP_INFO(this->get_logger(), "\n%d\t%f\t%f\t\t%f\t%f\t%f", j, cones[j].x, cones[j].y, x, y, r);
                        cloud_cones->points.push_back(cloud_cropped->points[i]);
                        break;  //avoid duplicates (and unnecessary computations)
                    }
                }
            }
            output_cloud = cloud_cones;
        }
        
        // Convert to ROS data type
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*output_cloud, output_msg);
        // Add the same frame_id as the input, it is not included in pcl PointXYZI
        output_msg.header.frame_id = input_msg->header.frame_id;
        // Publish the data as a ROS message
        pub_lidar_->publish(output_msg);
        RCLCPP_INFO_STREAM(this->get_logger(), output_cloud->points.size());
    }
/*
    void timer_callback()
    {
        sensor_msgs::msg::PointCloud2 test_msg;
        pcl::toROSMsg(test_cloud, test_msg);
        test_msg.header.frame_id = "laser_data_frame";
        rclcpp::Time now = this->get_clock()->now();
        test_msg.header.stamp = now;
        pub_testcloud_->publish(test_msg);
    }
*/
    void cam_cones_callback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr markers_in)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "First CamCone input recieved, CamFilter is turned %s",
            (toggle_cam_filter ? "ON" : "OFF"));
        if (!toggle_cam_filter) return;

        int s = markers_in->markers.size();
        cones.clear();
        pcl::PointXY tempoint;
        for (int i = 0; i < s; i++)
        {
            tempoint.x = -markers_in->markers[i].pose.position.x;   //todo: proper transform
            tempoint.y = -markers_in->markers[i].pose.position.y;
            if ((tempoint.x < -0.001 || tempoint.x > +0.001) &&
                (tempoint.y < -0.001 || tempoint.y > +0.001))
                cones.push_back(tempoint);
            //RCLCPP_INFO(this->get_logger(), "\n%d\t%f\t%f", i, tempoint.x, tempoint.y);
        }
        //RCLCPP_INFO(this->get_logger(), "---");
        //timer_callback(); //temp
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
    //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_testcloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_cam_cones_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    std::string cloud_in_topic = "pointcloud_topic";
    std::string cam_cones_topic = "cone_coordinates";
    float minX_over = -10.0, minY_over = -5.0, minZ_over = -2.0;
    float maxX_over = +10.0, maxY_over = +5.0, maxZ_over = -0.15;
    float minX_vehicle = -5.0, minY_vehicle = -5.0;
    float maxX_vehicle = +5.0, maxY_vehicle = +5.0;
    std::vector<double> crop_box_array = {-0.8, 0.8, -0.8, 0.8, -0.8, 0.8};
    float cam_cone_radius = 0.5;
    bool verbose1 = false;
    bool verbose2 = true;
    bool toggle_box_filter = true;
    bool toggle_cam_filter = false;
    size_t count_;
    const char* ERROR_TEXT_PARAM_NUM = "ERROR: Incorrect number of Crop Box parameters!"
                " (should be a multiple of 6: minX, maxX, minY, maxY, minZ, maxZ)";

    void printcfg()
    {
        int s = crop_box_array.size();
        if (s % 6)
            RCLCPP_ERROR(this->get_logger(), ERROR_TEXT_PARAM_NUM);
        else
        {   
            std::string tempstr = "BoxFilter parameters:"
                "\n#\t minX\t maxX\t minY\t maxY\t minZ\t maxZ";
            char* buff = new char[52];  //line length is 52 in this format
            for (int i = 0; i < s; i += 6)
            {
                snprintf(buff, 52,
                    "\n%d\t%+5.2f\t%+5.2f\t%+5.2f\t%+5.2f\t%+5.2f\t%+5.2f",
                    i / 6,
                    crop_box_array[i],     //minX
                    crop_box_array[i+1],   //maxX
                    crop_box_array[i+2],   //minY
                    crop_box_array[i+3],   //maxY
                    crop_box_array[i+4],   //minZ
                    crop_box_array[i+5]);  //maxZ
                    tempstr += buff;
            }
            tempstr += "\n-----";
            RCLCPP_INFO_STREAM(this->get_logger(), tempstr);
            delete[] buff;
        }
    }
    std::vector<pcl::PointXY> cones;
    //pcl::PointCloud<pcl::PointXYZI> test_cloud;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FliterVehicle>());
    rclcpp::shutdown();
    return 0;
}