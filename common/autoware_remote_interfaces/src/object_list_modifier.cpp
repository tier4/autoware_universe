/**
 * @file object_list_modifier.cpp
 * @brief Node for modifying the predicted object list of Autoware.
 * @copyright 2025 TUM-FTM
 * @ingroup autoware_remote_interfaces
 */

#include "autoware_remote_interfaces/object_list_modifier.hpp"

namespace autoware_remote_interfaces {

// Gate state machine implementation
// Actions
auto ac_set_active_object_ids = [](ObjectListModifier* gate) { gate->set_active_object_ids(); };
auto reset_active_object_ids = [](ObjectListModifier* gate) { gate->reset_active_object_ids(); };

struct StateMachine { 
    // Derived class as context for the state machine
    using context_type = ObjectListModifier;
    // States
    struct ForwardingObjects {};
    struct RemovingObjects {};
    struct SettingObjectsStatic {};
    struct SettingObjectsUnkown {};
    // Events
    struct START_REMOVE {};
    struct START_SET_STATIC {};
    struct START_SET_UNKOWN {};
    struct STOP_MOD {};

    auto operator()() const {
        using namespace boost::sml;
        return make_transition_table(
            *state<ForwardingObjects> + event<START_REMOVE> / ac_set_active_object_ids = state<RemovingObjects>,
             state<RemovingObjects> + event<START_REMOVE> / ac_set_active_object_ids = state<RemovingObjects>,
             state<RemovingObjects> + event<START_SET_STATIC> / ac_set_active_object_ids = state<SettingObjectsStatic>,
             state<RemovingObjects> + event<START_SET_UNKOWN> / ac_set_active_object_ids = state<SettingObjectsUnkown>,

             state<ForwardingObjects> + event<START_SET_STATIC> / ac_set_active_object_ids = state<SettingObjectsStatic>,
             state<SettingObjectsStatic> + event<START_SET_STATIC> / ac_set_active_object_ids = state<SettingObjectsStatic>,
             state<SettingObjectsStatic> + event<START_REMOVE> / ac_set_active_object_ids = state<RemovingObjects>,
             state<SettingObjectsStatic> + event<START_SET_UNKOWN> / ac_set_active_object_ids = state<SettingObjectsUnkown>,

             state<ForwardingObjects> + event<START_SET_UNKOWN> / ac_set_active_object_ids = state<SettingObjectsUnkown>,
             state<SettingObjectsUnkown> + event<START_SET_UNKOWN> / ac_set_active_object_ids = state<SettingObjectsUnkown>,
             state<SettingObjectsUnkown> + event<START_REMOVE> / ac_set_active_object_ids = state<RemovingObjects>,
             state<SettingObjectsUnkown> + event<START_SET_STATIC> / ac_set_active_object_ids = state<SettingObjectsStatic>,
            
             // Reset
             state<RemovingObjects> + event<STOP_MOD> / reset_active_object_ids= state<ForwardingObjects>,
             state<SettingObjectsStatic> + event<STOP_MOD> / reset_active_object_ids = state<ForwardingObjects>,
             state<SettingObjectsUnkown> + event<STOP_MOD> / reset_active_object_ids = state<ForwardingObjects>
        );
    };
};   

// Modifier node implementation
ObjectListModifier::ObjectListModifier() : BaseModifier<StateMachine>("object_list_modifier", this),
   incoming_object_ids_(),
   active_object_ids_()
{
    // Parameter
    declare_parameter("input_topic", "predicted_objects");
    declare_parameter("output_topic", "mod_predicted_objects");
    declare_parameter("select_mod_service", "mod_predicted_objects");

    std::string input_topic, select_mod_service, output_topic;
    get_parameter("input_topic", input_topic);
    get_parameter("output_topic", output_topic);
    get_parameter("select_mod_service", select_mod_service);
    
    // Add input subscribers
    add_input_subscriber<autoware_perception_msgs::msg::PredictedObjects>(
        input_topic,
        [this](const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg) {
               callback_input_object_list(msg);
        });
    
    // Add control service
    set_control_service<ModObjectList>(
        select_mod_service,
        [this](const ModObjectList::Request::SharedPtr request, ModObjectList::Response::SharedPtr response) {
               callback_control(request, response);
        });

    // Add output publisher
    set_output_publisher<autoware_perception_msgs::msg::PredictedObjects>(output_topic);

    // Log init
    RCLCPP_INFO(get_logger(), "%s started!", get_name());
};

// Input callbacks 
void ObjectListModifier::callback_input_object_list(const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg) 
{
    if (state_machine_->is(sml::state<StateMachine::ForwardingObjects>))
    {
        publish_to_output(msg);
    }
    else if(state_machine_->is(sml::state<StateMachine::RemovingObjects>))
    {
        autoware_perception_msgs::msg::PredictedObjects::SharedPtr modified_msg = 
            std::make_shared<autoware_perception_msgs::msg::PredictedObjects>(*msg);
        
        modified_msg->objects.erase(
            std::remove_if(modified_msg->objects.begin(), 
                           modified_msg->objects.end(), 
                           [this](const autoware_perception_msgs::msg::PredictedObject &obj) 
                           {
                           return std::find(active_object_ids_.begin(), 
                                            active_object_ids_.end(), 
                                            obj.object_id) != active_object_ids_.end();
                           }), 
                           modified_msg->objects.end());
        publish_to_output(modified_msg);
    }
    else if(state_machine_->is(sml::state<StateMachine::SettingObjectsStatic>))
    {
        autoware_perception_msgs::msg::PredictedObjects::SharedPtr modified_msg = 
            std::make_shared<autoware_perception_msgs::msg::PredictedObjects>(*msg);

        autoware_perception_msgs::msg::PredictedPath static_path;
        static_path.confidence = 1.0;
        static_path.time_step.sec = 0.0;
        static_path.time_step.nanosec = 500000000;
        
        std::for_each(modified_msg->objects.begin(), 
                      modified_msg->objects.end(),
                      [this, &static_path](autoware_perception_msgs::msg::PredictedObject &obj) {
                            if (std::find(active_object_ids_.begin(), active_object_ids_.end(), obj.object_id) 
                                != active_object_ids_.end()) 
                            {
                                obj.kinematics.predicted_paths.clear();
                                static_path.path.clear();
                                
                                for (int i = 0; i < 11; ++i) {
                                    static_path.path.emplace_back(obj.kinematics.initial_pose_with_covariance.pose);
                                }
                                obj.kinematics.predicted_paths.emplace_back(static_path);
                            }
                    });
                    
        publish_to_output(modified_msg);
    }
    else if(state_machine_->is(sml::state<StateMachine::SettingObjectsUnkown>))
    {
        autoware_perception_msgs::msg::ObjectClassification unkown;
        unkown.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
        unkown.probability = 1.0;

        autoware_perception_msgs::msg::PredictedObjects::SharedPtr modified_msg = 
            std::make_shared<autoware_perception_msgs::msg::PredictedObjects>(*msg);
            std::for_each(modified_msg->objects.begin(), 
            modified_msg->objects.end(),
            [this, &unkown](autoware_perception_msgs::msg::PredictedObject &obj) {
                  if (std::find(active_object_ids_.begin(), active_object_ids_.end(), obj.object_id) 
                      != active_object_ids_.end()) 
                  {
                      obj.classification.clear();
                      obj.classification.push_back(unkown);
                  }
          });

        publish_to_output(modified_msg);
    }
};

// Control service callback
void ObjectListModifier::callback_control(const ModObjectList::Request::SharedPtr request, ModObjectList::Response::SharedPtr response) 
{
    bool success{false};
    
    if (request->modification == ModObjectList::Request::MOD_STOP_MOD)
    {
        if (state_machine_->process_event(StateMachine::STOP_MOD()))
        {
            RCLCPP_INFO(get_logger(), "State: ForwardingObjects");
            success = true;
        }
    }
    else if (request->modification == ModObjectList::Request::MOD_REMOVE)
    {
        incoming_object_ids_ = request->object_ids;
        if (state_machine_->process_event(StateMachine::START_REMOVE()))
        {
            RCLCPP_INFO(get_logger(), "State: RemovingObjects");
            success = true;
        }
    }
    else if (request->modification == ModObjectList::Request::MOD_SET_STATIC)
    {
        incoming_object_ids_ = request->object_ids;
        if (state_machine_->process_event(StateMachine::START_SET_STATIC()))
        {
            RCLCPP_INFO(get_logger(), "State: SettingObjectsStatic");
            success = true;
        }
    }
    else if (request->modification == ModObjectList::Request::MOD_SET_UNKNOWN)
    {
        incoming_object_ids_ = request->object_ids;
        if (state_machine_->process_event(StateMachine::START_SET_UNKOWN()))
        {
            RCLCPP_INFO(get_logger(), "State: SettingObjectsUnkown");
            success = true;
        }
    }

    response->set__success(success);
};

// Functions
void ObjectListModifier::set_active_object_ids() { active_object_ids_ = incoming_object_ids_; };

void ObjectListModifier::reset_active_object_ids(){ active_object_ids_.clear(); };

}; // namespace autoware_remote_interfaces