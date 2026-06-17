/**
 * @file generic_multiplexer_node.cpp
 * @brief Multiplexer node to choose an input topics to pass to an output topic.
 * @copyright 2025 TUM-FTM
 * @ingroup autoware_remote_interfaces
 */

 #include <rclcpp/rclcpp.hpp>

 #include "autoware_remote_interfaces/generic_multiplexer.hpp"
 
 int main(int argc, char **argv) 
 {
     rclcpp::init(argc, argv);
     auto generic_multiplexer = std::make_shared<autoware_remote_interfaces::GenericMultiplexer>();
     rclcpp::spin(generic_multiplexer);
     rclcpp::shutdown();
     return 0;
 }