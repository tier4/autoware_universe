// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VIRTUAL_TRAFFIC_LIGHT_PANEL_HPP_
#define VIRTUAL_TRAFFIC_LIGHT_PANEL_HPP_

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <tier4_v2x_msgs/msg/infrastructure_command.hpp>
#include <tier4_v2x_msgs/msg/infrastructure_command_array.hpp>
#include <tier4_v2x_msgs/msg/virtual_traffic_light_state_array.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::virtual_traffic_light_rviz_plugin
{

using tier4_v2x_msgs::msg::InfrastructureCommandArray;
using tier4_v2x_msgs::msg::VirtualTrafficLightStateArray;

struct ReceivedCommand
{
  std::string type;
  std::string id;
  std::string lane_id;
  uint8_t state{tier4_v2x_msgs::msg::InfrastructureCommand::NONE};
  //! Index in the last command array (route order for the trajectory modifier).
  size_t route_order{0};
  std::chrono::steady_clock::time_point received_time;
};

//! Commands written by the subscription callback and read by the panel timer, keyed by type and id.
struct ReceivedCommands
{
  std::mutex mutex;
  std::map<std::string, ReceivedCommand, std::less<>> by_key;
};

struct Answer
{
  bool approval{false};
  bool is_finalized{false};
};

class VirtualTrafficLightPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit VirtualTrafficLightPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void on_timer();
  void on_input_topic_edited();
  void on_output_topic_edited();
  void on_publish_toggled(bool checked);

private:
  void create_subscription();
  void create_publisher();
  rclcpp::Subscription<InfrastructureCommandArray>::SharedPtr make_subscription(
    const std::string & topic) const;
  std::vector<ReceivedCommand> take_active_commands();
  void update_table(const std::vector<ReceivedCommand> & commands);
  void publish_states(const std::vector<ReceivedCommand> & commands);
  QLabel * cell_label(int row, int column) const;
  void keep_answer_columns_visible();
  QPushButton * make_answer_button(const std::string & key, bool is_finalization, bool value);

  QPushButton * publish_button_;
  QLineEdit * input_topic_edit_;
  QLineEdit * output_topic_edit_;
  QLabel * status_label_;
  QTableWidget * table_;
  QTimer * timer_;

  rclcpp::Node::SharedPtr raw_node_;
  std::shared_ptr<ReceivedCommands> received_commands_{std::make_shared<ReceivedCommands>()};
  rclcpp::Subscription<InfrastructureCommandArray>::SharedPtr sub_commands_;
  rclcpp::Publisher<VirtualTrafficLightStateArray>::SharedPtr pub_states_;
  //! Empty unless the last topic change was rejected; the previous topic stays in use.
  QString input_topic_error_;
  QString output_topic_error_;

  //! Keys of the rows currently shown, in row order.
  std::vector<std::string> row_keys_;
  //! Kept for keys that left the table so that the choice is restored when they come back.
  std::map<std::string, Answer, std::less<>> answers_;
};

}  // namespace autoware::virtual_traffic_light_rviz_plugin

#endif  // VIRTUAL_TRAFFIC_LIGHT_PANEL_HPP_
