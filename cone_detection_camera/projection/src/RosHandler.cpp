#include "RosHandler.h"

RosHandler::RosHandler() : Node("cone_projection"){
    this->sub_coords = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "cone_coordinates", 1, std::bind(&RosHandler::float32_callback, this,std::placeholders::_1)
    );
    
    this->pub_coords = this->create_publisher<std_msgs::msg::Float32MultiArray>("pub_proj_coords", 1);
    this->pub_marker_array_yellow = this->create_publisher<visualization_msgs::msg::MarkerArray>("yellow_cones", 1);
    this->pub_marker_array_blue = this->create_publisher<visualization_msgs::msg::MarkerArray>("blue_cones", 1);

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
    
    visualization_msgs::msg::MarkerArray markerArrayBlue;
    visualization_msgs::msg::MarkerArray markerArrayYellow;
    visualization_msgs::msg::Marker markerPoint;
    std_msgs::msg::Float32MultiArray pub_msg;
    markerPoint.scale.x = 0.2;
    markerPoint.scale.y = 0.2;
    markerPoint.scale.z = 0.5;
    markerPoint.color.a = 1.0;
    int yellow_id = 1;
    int blue_id = 2;
    
    for(unsigned int i=0; i<msg.layout.dim.size(); i++){
        ProjectionHandler::get().doProjection(new Cone(int(msg.data[i*5]), int(msg.data[i*5+1]), int(msg.data[i*5+2]), msg.data[i*5+3]));
        //ProjectionHandler::get().doProjection(new Cone(1, 157, 324, 30.0f));

        // This check is enough
        if(!ProjectionHandler::get().getCurParticle()) { return; }

        pub_msg.data.push_back(ProjectionHandler::get().getCurParticle()->getId());
        pub_msg.data.push_back(ProjectionHandler::get().getCurParticle()->getPosition().getX());
        pub_msg.data.push_back(ProjectionHandler::get().getCurParticle()->getPosition().getY());

        markerPoint.id = ProjectionHandler::get().getCurParticle()->getId();

        markerPoint.action = visualization_msgs::msg::Marker::MODIFY;
        markerPoint.type = visualization_msgs::msg::Marker::CYLINDER;
        markerPoint.header.frame_id = "base_link";
        markerPoint.pose.position.x = ProjectionHandler::get().getCurParticle()->getPosition().getX()/100;
        markerPoint.pose.position.y = ProjectionHandler::get().getCurParticle()->getPosition().getY()/100;
        markerPoint.pose.position.z = 0.0;
        markerPoint.lifetime = rclcpp::Duration::from_seconds(0.1);

        if(ProjectionHandler::get().getCurParticle()->getId()==1){
            markerPoint.color.r = 1.0;
            markerPoint.color.g = 1.0;
            markerPoint.color.b = 0.3;
            markerPoint.id = yellow_id;
            yellow_id += 2;
            markerArrayYellow.markers.push_back(markerPoint);
        }else if(ProjectionHandler::get().getCurParticle()->getId()==2){
            markerPoint.color.r = 0.0;
            markerPoint.color.g = 0.0;
            markerPoint.color.b = 1.0;
            markerPoint.id += blue_id;
            blue_id += 2;
            markerArrayBlue.markers.push_back(markerPoint);
        }
    }

    pub_coords->publish(pub_msg);
    this->pub_marker_array_yellow->publish(markerArrayYellow);
    this->pub_marker_array_blue->publish(markerArrayBlue);

    pub_msg.data.clear();
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