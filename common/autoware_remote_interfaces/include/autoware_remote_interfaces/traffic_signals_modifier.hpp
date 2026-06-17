/**
 * @file object_list_modifier.hpp
 * @brief Node for modifying the predicted object list of Autoware.
 * @copyright 2025 TUM-FTM
 * @ingroup autoware_remote_interfaces
 */

#pragma once

#include "boost/sml.hpp"

#include "autoware_remote_interfaces/base_modifier.hpp"

#include "tum_autoware_teleoperation_msgs/srv/mod_traffic_signals.hpp"

#include "autoware_perception_msgs/msg/traffic_light_group_array.hpp"

using tum_autoware_teleoperation_msgs::srv::ModTrafficSignals;

namespace autoware_remote_interfaces {

struct StateMachine;

class TrafficSignalsModifier : public autoware_remote_interfaces::BaseModifier<StateMachine>
{
    public:
        // Constructor
        TrafficSignalsModifier();
        // Destructor
        ~TrafficSignalsModifier() override = default;

        // Functions to be called by state machine

        // Variables

    private:
        // Input callbacks
        void callback_traffic_signals_message(const autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr msg);

        // Control service callback
        void callback_select_mod_service(const ModTrafficSignals::Request::SharedPtr request, 
                                         ModTrafficSignals::Response::SharedPtr response);
};

}; // namespace autoware_remote_interfaces