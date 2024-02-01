#include "RosHandler.h"

RosHandler::RosHandler() : Node("cone_projection"){
    this->sub_coords = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "cone_coordinates", 1, std::bind(&RosHandler::float32_callback, this,std::placeholders::_1)
    );
    this->pub_coords = this->create_publisher<std_msgs::msg::Float32MultiArray>("pub_proj_coords", 1);

    // param_subscriber_ = std::make_shared<rclcpp::ParameterEvenHandler>(this);

    // cb_handle_ = param_subscriber_->add_parameter_callback("display", callback_display);

    // dynamic_reconfigure::Server<my_dyn_rec::ValidateConfig> server;
    // dynamic_reconfigure::Server<my_dyn_rec::ValidateConfig>::CallbackType f;

    // f = boost::bind(&RosHandler::paramsCallback, this, _1, _2);
    // server.setCallback(f);

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

// void RosHandler::paramsCallback(my_dyn_rec::ValidateConfig &config, uint32_t level)
// {
//     CheckingVariables::HORIZON_HEIGHT = config.horizon_height;
//     CheckingVariables::FACTOR_HORIZONTAL = config.factor_horizontal;
//     CheckingVariables::FACTOR_VERTICAL = config.factor_vertical;
//     CheckingVariables::BASE_WIDTH = config.base_width;
//     CheckingVariables::CONE_MIN_AREA = config.min_cone_area;
//     CheckingVariables::DIST_MAX = config.max_cone_distance;
//     CheckingVariables::DISPLAY_INFO = config.display;
//     CheckingVariables::getInstance()->updateParams();
//     // ROS_INFO_STREAM("\nUpdated params:\n" << "HORIZON HEIGHT: " << CheckingVariables::HORIZON_HEIGHT << '\n'
//     //                 << "FACTOR HORIZONTAL: " << CheckingVariables::FACTOR_HORIZONTAL << '\n'
//     //                 << "FACTOR VERTICAL" << CheckingVariables::FACTOR_VERTICAL << '\n'
//     //                 << "BASE WIDTH: " << CheckingVariables::BASE_WIDTH << '\n'
//     //                 << "MINIMUM AREA OF CONE: " << CheckingVariables::CONE_MIN_AREA << '\n'
//     //                 << "MAXIMUM DISTANCE: " << CheckingVariables::DIST_MAX << '\n'
//     //                 << "HORIZON_HEIGHT_REL: " << CheckingVariables::getInstance()->HORIZON_HEIGHT_REL << '\n'
//     //                 << "DISPLAY INFO: " << CheckingVariables::getInstance()->DISPLAY_INFO << '\n');
// }