#include "RosHandler.h"
#include "CheckingVariables.h"

int main(int argc, char** argv) 
{
    
    CheckingVariables* Variables = CheckingVariables::getInstance();
    rclcpp::init(argc, argv);
    // ROS_INFO_STREAM("Node started: " << ros::this_node::getName());

    RosHandler* r_handler = new RosHandler();

    rclcpp::spin(std::make_shared<RosHandler>());

    delete r_handler;
    delete Variables;
	return 0;
}