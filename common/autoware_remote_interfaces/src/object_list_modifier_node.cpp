/**
 * @file object_list_modifier_node.cpp
 * @brief Node for modifying the predicted object list of Autoware.
 * @copyright 2025 TUM-FTM
 * @ingroup autoware_remote_interfaces
 */

#include <rclcpp/rclcpp.hpp>

#include "autoware_remote_interfaces/object_list_modifier.hpp"

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto object_list_modifier_node = std::make_shared<autoware_remote_interfaces::ObjectListModifier>();
    rclcpp::spin(object_list_modifier_node);
    rclcpp::shutdown();
    return 0;
}