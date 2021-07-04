#include <memory>

#include "airsim_ros/ros_wrapper.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::string host_ip;
    auto node = rclcpp::Node::make_shared("airsim_ros_wrapper");
    // Declare parameters
    node->declare_parameter<std::string>("host_ip", std::getenv("WSL_HOST_IP"));
    // Load parameters
    node->get_parameter("host_ip", host_ip);

    auto wrapper = std::make_shared<airsim_ros::ROSWrapper>(node, host_ip);
    rclcpp::spin(wrapper->getNode());
    rclcpp::shutdown();

    return 0;
}
