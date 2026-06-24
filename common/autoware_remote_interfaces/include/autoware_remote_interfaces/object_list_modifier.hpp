/**
 * @file object_list_modifier.hpp
 * @brief Node for modifying the predicted object list of Autoware.
 * @copyright 2025 TUM-FTM
 * @ingroup autoware_remote_interfaces
 */

#pragma once

#include "boost/sml.hpp"

#include "autoware_remote_interfaces/base_modifier.hpp"

#include "unique_identifier_msgs/msg/uuid.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"

#include "autoware_adapi_v1_msgs/srv/mod_object_list.hpp"

using autoware_adapi_v1_msgs::srv::ModObjectList;

namespace autoware_remote_interfaces {

struct StateMachine;

class ObjectListModifier : public autoware_remote_interfaces::BaseModifier<StateMachine>
{
    public:
        // Constructor
        ObjectListModifier();
        // Destructor
        ~ObjectListModifier() override = default;

        // Functions to be called by state machine
        void set_active_object_ids();
        void reset_active_object_ids();

        // Variables
        std::vector<unique_identifier_msgs::msg::UUID> incoming_object_ids_;
        std::vector<unique_identifier_msgs::msg::UUID> active_object_ids_;

    private:
        // Input callbacks
        void callback_input_object_list(const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg);

        // Control service callback
        void callback_control(const ModObjectList::Request::SharedPtr request, 
                                    ModObjectList::Response::SharedPtr response);
};

}; // namespace autoware_remote_interfaces