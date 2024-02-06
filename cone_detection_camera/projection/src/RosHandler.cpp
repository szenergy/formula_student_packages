#include "RosHandler.h"

RosHandler::RosHandler() : Node("cone_projection"){
    this->sub_coords = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "cone_coordinates", 1, std::bind(&RosHandler::float32_callback, this,std::placeholders::_1)
    );
    this->pub_coords = this->create_publisher<std_msgs::msg::Float32MultiArray>("pub_proj_coords", 1);

    this->declare_parameter<int>("horizon_height", CheckingVariables::HORIZON_HEIGHT);
    this->declare_parameter<float>("factor_horizontal", CheckingVariables::FACTOR_HORIZONTAL);
    this->declare_parameter<float>("factor_vertical", CheckingVariables::FACTOR_VERTICAL);
    this->declare_parameter<float>("base_width", CheckingVariables::BASE_WIDTH);
    this->declare_parameter<float>("cone_min_area", CheckingVariables::CONE_MIN_AREA);
    this->declare_parameter<float>("dist_max", CheckingVariables::DIST_MAX);
    this->declare_parameter<bool>("display", CheckingVariables::DISPLAY_INFO);
    
    this->get_parameter("horizon_height", CheckingVariables::HORIZON_HEIGHT);
    this->get_parameter("factor_horizontal", CheckingVariables::FACTOR_HORIZONTAL);
    this->get_parameter("factor_vertical", CheckingVariables::FACTOR_VERTICAL);
    this->get_parameter("base_width", CheckingVariables::BASE_WIDTH);
    this->get_parameter("cone_min_area", CheckingVariables::CONE_MIN_AREA);
    this->get_parameter("dist_max", CheckingVariables::DIST_MAX);
    this->get_parameter("display", CheckingVariables::DISPLAY_INFO);

    

    cb_handle_ = this->add_on_set_parameters_callback(std::bind(&RosHandler::parametersCallback, this, std::placeholders::_1));
    
    // rclcpp::spin(std::make_shared<RosHandler>());
}

void RosHandler::float32_callback(const std_msgs::msg::Float32MultiArray& msg){
    
    ProjectionHandler::get().doProjection(new Cone(int(msg.data[0]), int(msg.data[1]), int(msg.data[2]), msg.data[3]));
    //ProjectionHandler::get().doProjection(new Cone(1, 157, 324, 30.0f));

    // This check is enough
    if(!ProjectionHandler::get().getCurParticle()) { return; }

    std_msgs::msg::Float32MultiArray pub_msg;
    pub_msg.data.clear();
    pub_msg.data.push_back(ProjectionHandler::get().getCurParticle()->getId());
    pub_msg.data.push_back(ProjectionHandler::get().getCurParticle()->getPosition().getX());
    pub_msg.data.push_back(ProjectionHandler::get().getCurParticle()->getPosition().getY());

    pub_coords->publish(pub_msg);

    //ROS_INFO_STREAM("ID: " << msg.header.frame_id << " X: " << msg.point.x << " Y: " << msg.point.y << " Area: " << msg.point.z);    
    //ROS_INFO_STREAM("ID: " << p->getId() << " X: " << p->getPosition().getX() << " Y: " << p->getPosition().getY());    
}


rcl_interfaces::msg::SetParametersResult RosHandler::parametersCallback(const std::vector<rclcpp::Parameter> &parameters){
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &param: parameters){
            if(param.get_name()=="display"){
                CheckingVariables::DISPLAY_INFO = param.as_bool();
            }else if(param.get_name()=="horizon_height"){
                CheckingVariables::HORIZON_HEIGHT = param.as_int();
            }else if(param.get_name()=="factor_horizontal"){
                CheckingVariables::FACTOR_HORIZONTAL = param.as_double();
            }else if(param.get_name()=="factor_vertical"){
                CheckingVariables::FACTOR_VERTICAL = param.as_double();
            }else if(param.get_name()=="base_width"){
                CheckingVariables::BASE_WIDTH = param.as_double();
            }else if(param.get_name()=="cone_min_area"){
                CheckingVariables::CONE_MIN_AREA = param.as_double();
            }else if(param.get_name()=="dist_max"){
                CheckingVariables::DIST_MAX = param.as_double();
            }
            RCLCPP_INFO(this->get_logger(), "%s", param.get_name().c_str());
            RCLCPP_INFO(this->get_logger(), "%s", param.get_type_name().c_str());
            RCLCPP_INFO(this->get_logger(), "%s", param.value_to_string().c_str());
        }

        CheckingVariables::getInstance()->updateParams();
        // Here update class attributes, do some actions, etc.
        return result;
}