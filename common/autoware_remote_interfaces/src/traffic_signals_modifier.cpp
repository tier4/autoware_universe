/**
 * @file object_list_modifier.cpp
 * @brief Node for modifying the predicted object list of Autoware.
 * @copyright 2025 TUM-FTM
 * @ingroup autoware_remote_interfaces
 */

#include "autoware_remote_interfaces/traffic_signals_modifier.hpp"

namespace autoware_remote_interfaces {

// Gate state machine implementation
struct StateMachine { 
    // Derived class as context for the state machine
    using context_type = TrafficSignalsModifier;
    // States
    struct NoMod {};
    struct SettingAllGreen {};
    struct SettingAllRed {};

    // Events
    struct NO_MOD {};
    struct SET_GREEN {};
    struct SET_RED {};

    auto operator()() const {
        using namespace boost::sml;
        return make_transition_table(
            // Enable
            *state<NoMod> + event<SET_GREEN>  = state<SettingAllGreen>,
             state<NoMod> + event<SET_RED>  = state<SettingAllRed>,
             // Disable
             state<SettingAllGreen> + event<NO_MOD>  = state<NoMod>,
             state<SettingAllRed> + event<NO_MOD>  = state<NoMod>,
             // Change
             state<SettingAllGreen> + event<SET_RED>  = state<SettingAllRed>,
             state<SettingAllRed> + event<SET_GREEN>  = state<SettingAllGreen>,
             // Update
             state<NoMod> + event<NO_MOD>  = state<NoMod>,
             state<SettingAllGreen> + event<SET_GREEN>  = state<SettingAllGreen>,
             state<SettingAllRed> + event<SET_RED>  = state<SettingAllRed>
        );
    };
};   

// Modifier node implementation
TrafficSignalsModifier::TrafficSignalsModifier() : BaseModifier<StateMachine>("traffic_signals_modifier", this)
{
    // Parameter
    declare_parameter("input_topic", "traffic_signals");
    declare_parameter("output_topic", "mod_traffic_signals");
    declare_parameter("select_mod_service", "mod_traffic_signals");

    std::string input_topic, select_mod_service, output_topic;
    get_parameter("input_topic", input_topic);
    get_parameter("output_topic", output_topic);
    get_parameter("select_mod_service", select_mod_service);
    
    // Add input subscribers
    add_input_subscriber<autoware_perception_msgs::msg::TrafficLightGroupArray>(
        input_topic,
        [this](const autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr msg) {
            callback_traffic_signals_message(msg);
        });
    
    // Add control service
    set_control_service<ModTrafficSignals>(
        select_mod_service,
        [this](const ModTrafficSignals::Request::SharedPtr request, ModTrafficSignals::Response::SharedPtr response) {
            callback_select_mod_service(request, response);
        });

    // Add output publisher
    set_output_publisher<autoware_perception_msgs::msg::TrafficLightGroupArray>(output_topic);

    // Log init
    RCLCPP_INFO(get_logger(), "%s started!", get_name());
};

// Input callbacks 
void TrafficSignalsModifier::callback_traffic_signals_message(const autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr msg) 
{
    if (state_machine_->is(sml::state<StateMachine::NoMod>))
    {
        publish_to_output(msg);
    }
    else if (state_machine_->is(sml::state<StateMachine::SettingAllGreen>))
    {
        autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr modified_msg 
            = std::make_shared<autoware_perception_msgs::msg::TrafficLightGroupArray>(*msg);

        for (auto &traffic_light_group: modified_msg->traffic_light_groups)
        {
            for (auto &element: traffic_light_group.elements)
            {
                element.set__confidence(1.0f);
                element.set__status(autoware_perception_msgs::msg::TrafficLightElement::SOLID_ON);
                element.color = autoware_perception_msgs::msg::TrafficLightElement::GREEN;
            }
        }
        publish_to_output(modified_msg);
    }
    else if (state_machine_->is(sml::state<StateMachine::SettingAllRed>))
    {
        autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr modified_msg 
            = std::make_shared<autoware_perception_msgs::msg::TrafficLightGroupArray>(*msg);

        for (auto &traffic_light_group: modified_msg->traffic_light_groups)
        {
            for (auto &element: traffic_light_group.elements)
            {
                element.set__confidence(1.0f);
                element.set__status(autoware_perception_msgs::msg::TrafficLightElement::SOLID_ON);
                element.color = autoware_perception_msgs::msg::TrafficLightElement::RED;
            }
        }
        publish_to_output(modified_msg);
    }
};

// Control service callback
void TrafficSignalsModifier::callback_select_mod_service(const ModTrafficSignals::Request::SharedPtr request, ModTrafficSignals::Response::SharedPtr response) 
{
    bool success{false};

    if (request->modification == ModTrafficSignals::Request::MOD_NO_MOD)
    {
        if (state_machine_->process_event(StateMachine::NO_MOD()))
        {
            RCLCPP_INFO(get_logger(), "State: NoMod");
            success = true;
        }
    }
    else if (request->modification == ModTrafficSignals::Request::MOD_SET_GREEN) 
    {
        if (state_machine_->process_event(StateMachine::SET_GREEN()))
        {
            RCLCPP_INFO(get_logger(), "State: SettingAllGreen");
            success = true;
        }
    }
    else if (request->modification == ModTrafficSignals::Request::MOD_SET_RED) 
    {
        if (state_machine_->process_event(StateMachine::SET_RED()))
        {
            RCLCPP_INFO(get_logger(), "State: SettingAllRed");
            success = true;
        }
    }

    response->set__success(success);
};

// Functions


}; // namespace autoware_remote_interfaces