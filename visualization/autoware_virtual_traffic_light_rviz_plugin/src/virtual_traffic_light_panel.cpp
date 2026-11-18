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

#include "virtual_traffic_light_panel.hpp"

#include <QGridLayout>
#include <QHeaderView>
#include <QScrollBar>
#include <QVBoxLayout>
#include <rclcpp/exceptions.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

#include <tier4_v2x_msgs/msg/infrastructure_command.hpp>
#include <tier4_v2x_msgs/msg/key_value.hpp>
#include <tier4_v2x_msgs/msg/virtual_traffic_light_state.hpp>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace autoware::virtual_traffic_light_rviz_plugin
{
namespace
{
using tier4_v2x_msgs::msg::InfrastructureCommand;
using tier4_v2x_msgs::msg::KeyValue;
using tier4_v2x_msgs::msg::VirtualTrafficLightState;

constexpr double STALE_TIMEOUT_S = 3.0;
constexpr int TIMER_PERIOD_MS = 100;

constexpr int COLUMN_ID = 0;
constexpr int COLUMN_STATE = 1;
constexpr int COLUMN_APPROVAL = 2;
constexpr int COLUMN_FINALIZED = 3;
constexpr int COLUMN_LANE = 4;
constexpr int COLUMN_TYPE = 5;
constexpr int COLUMN_COUNT = 6;

const char * const DEFAULT_INPUT_TOPIC =
  "/planning/scenario_planning/status/infrastructure_commands";
const char * const DEFAULT_OUTPUT_TOPIC = "/system/v2x/virtual_traffic_light_states";

std::string make_key(const std::string & type, const std::string & id)
{
  return type + "-" + id;
}

std::string find_tag(const std::vector<KeyValue> & tags, const std::string & key)
{
  for (const auto & tag : tags) {
    if (tag.key == key) {
      return tag.value;
    }
  }
  return "-";
}

QString to_state_name(const uint8_t state)
{
  switch (state) {
    case InfrastructureCommand::NONE:
      return "NONE";
    case InfrastructureCommand::REQUESTING:
      return "REQUESTING";
    case InfrastructureCommand::PASSING:
      return "PASSING";
    case InfrastructureCommand::FINALIZING:
      return "FINALIZING";
    case InfrastructureCommand::FINALIZED:
      return "FINALIZED";
    default:
      return QString::number(state);
  }
}

//! True while planning asks for the right of way or waits at the end line for finalization.
bool is_waiting_for_infrastructure(const uint8_t state, const Answer & answer)
{
  return state == InfrastructureCommand::REQUESTING ||
         (state == InfrastructureCommand::FINALIZING && !answer.is_finalized);
}

// Explicit colours so that the cells read the same in every rviz theme; a theme that styles
// QTableView::item ignores per-item colours, so the cells are widgets with their own style sheet.
const char * const CELL_STYLE = "QLabel { padding: 0px 4px; }";
const char * const HIGHLIGHTED_CELL_STYLE =
  "QLabel { background-color: #FFC878; color: black; padding: 0px 4px; }";
const char * const TRUE_STYLE =
  "QPushButton { background-color: #2E7D32; color: white; font-weight: bold; "
  "border-radius: 3px; padding: 2px 10px; }";
const char * const FALSE_STYLE =
  "QPushButton { background-color: #8E2B2B; color: white; font-weight: bold; "
  "border-radius: 3px; padding: 2px 10px; }";
const char * const PUBLISH_ON_STYLE =
  "QPushButton { background-color: #2E7D32; color: white; font-weight: bold; "
  "border-radius: 4px; padding: 6px; }";
const char * const PUBLISH_OFF_STYLE =
  "QPushButton { background-color: #555555; color: white; font-weight: bold; "
  "border-radius: 4px; padding: 6px; }";

void style_answer_button(QPushButton * button, const bool value)
{
  button->setText(value ? "TRUE" : "FALSE");
  button->setStyleSheet(value ? TRUE_STYLE : FALSE_STYLE);
}

void style_publish_button(QPushButton * button, const bool publishing)
{
  button->setText(publishing ? "PUBLISH ANSWERS: ON" : "PUBLISH ANSWERS: OFF");
  button->setStyleSheet(publishing ? PUBLISH_ON_STYLE : PUBLISH_OFF_STYLE);
}

QLabel * make_cell_label(const QString & text)
{
  auto * label = new QLabel(text);
  label->setStyleSheet(CELL_STYLE);
  return label;
}

void set_cell_style(QLabel * label, const char * const style)
{
  if (label->styleSheet() != style) {
    label->setStyleSheet(style);
  }
}
}  // namespace

VirtualTrafficLightPanel::VirtualTrafficLightPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  publish_button_ = new QPushButton();
  publish_button_->setCheckable(true);
  publish_button_->setChecked(false);
  publish_button_->setMinimumHeight(36);
  style_publish_button(publish_button_, false);
  layout->addWidget(publish_button_);

  auto * topic_layout = new QGridLayout();
  input_topic_edit_ = new QLineEdit(DEFAULT_INPUT_TOPIC);
  output_topic_edit_ = new QLineEdit(DEFAULT_OUTPUT_TOPIC);
  topic_layout->addWidget(new QLabel("Commands:"), 0, 0);
  topic_layout->addWidget(input_topic_edit_, 0, 1);
  topic_layout->addWidget(new QLabel("States:"), 1, 0);
  topic_layout->addWidget(output_topic_edit_, 1, 1);
  layout->addLayout(topic_layout);

  table_ = new QTableWidget(0, COLUMN_COUNT);
  table_->setHorizontalHeaderLabels({"ID", "State", "Approval", "Finalized", "Lane", "Type"});
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->verticalHeader()->setVisible(false);
  table_->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  table_->setSelectionMode(QAbstractItemView::NoSelection);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  layout->addWidget(table_);

  status_label_ = new QLabel();
  // Word wrap keeps the status text from resizing the dock whenever it changes.
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);

  setLayout(layout);

  timer_ = new QTimer(this);

  connect(
    input_topic_edit_, &QLineEdit::editingFinished, this,
    &VirtualTrafficLightPanel::on_input_topic_edited);
  connect(
    output_topic_edit_, &QLineEdit::editingFinished, this,
    &VirtualTrafficLightPanel::on_output_topic_edited);
  connect(
    publish_button_, &QPushButton::toggled, this, &VirtualTrafficLightPanel::on_publish_toggled);
  connect(timer_, &QTimer::timeout, this, &VirtualTrafficLightPanel::on_timer);
}

void VirtualTrafficLightPanel::onInitialize()
{
  raw_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  create_subscription();
  create_publisher();
  timer_->start(TIMER_PERIOD_MS);
}

void VirtualTrafficLightPanel::create_subscription()
{
  if (!raw_node_) {
    return;
  }
  const auto topic = input_topic_edit_->text();
  try {
    sub_commands_ = make_subscription(topic.toStdString());
    input_topic_error_.clear();
  } catch (const rclcpp::exceptions::NameValidationError &) {
    input_topic_error_ = QString("Invalid commands topic '%1'.").arg(topic);
    if (sub_commands_) {
      input_topic_edit_->setText(QString::fromStdString(sub_commands_->get_topic_name()));
    }
  }
}

rclcpp::Subscription<InfrastructureCommandArray>::SharedPtr
VirtualTrafficLightPanel::make_subscription(const std::string & topic) const
{
  // The callback owns the shared buffer, so it stays valid even if the panel is destroyed first.
  return raw_node_->create_subscription<InfrastructureCommandArray>(
    topic, rclcpp::QoS{10},
    [received = received_commands_](const InfrastructureCommandArray::ConstSharedPtr msg) {
      const auto now = std::chrono::steady_clock::now();
      std::lock_guard<std::mutex> lock(received->mutex);
      for (size_t i = 0; i < msg->commands.size(); ++i) {
        const auto & command = msg->commands[i];
        ReceivedCommand & entry = received->by_key[make_key(command.type, command.id)];
        entry.route_order = i;
        entry.type = command.type;
        entry.id = command.id;
        entry.lane_id = find_tag(command.custom_tags, "lane_id");
        entry.state = command.state;
        entry.received_time = now;
      }
    });
}

void VirtualTrafficLightPanel::create_publisher()
{
  if (!raw_node_) {
    return;
  }
  const auto topic = output_topic_edit_->text();
  try {
    pub_states_ = raw_node_->create_publisher<VirtualTrafficLightStateArray>(
      topic.toStdString(), rclcpp::QoS{1});
    output_topic_error_.clear();
  } catch (const rclcpp::exceptions::NameValidationError &) {
    output_topic_error_ = QString("Invalid states topic '%1'.").arg(topic);
    if (pub_states_) {
      output_topic_edit_->setText(QString::fromStdString(pub_states_->get_topic_name()));
    }
  }
}

void VirtualTrafficLightPanel::on_input_topic_edited()
{
  create_subscription();
}

void VirtualTrafficLightPanel::on_output_topic_edited()
{
  create_publisher();
}

void VirtualTrafficLightPanel::on_timer()
{
  const auto commands = take_active_commands();
  update_table(commands);
  publish_states(commands);
}

std::vector<ReceivedCommand> VirtualTrafficLightPanel::take_active_commands()
{
  const auto now = std::chrono::steady_clock::now();
  std::vector<ReceivedCommand> commands;
  std::lock_guard<std::mutex> lock(received_commands_->mutex);
  auto & by_key = received_commands_->by_key;
  for (auto it = by_key.begin(); it != by_key.end();) {
    const std::chrono::duration<double> age = now - it->second.received_time;
    if (age.count() > STALE_TIMEOUT_S) {
      it = by_key.erase(it);
      continue;
    }
    commands.push_back(it->second);
    ++it;
  }
  std::stable_sort(
    commands.begin(), commands.end(), [](const ReceivedCommand & a, const ReceivedCommand & b) {
      return a.route_order < b.route_order;
    });
  return commands;
}

void VirtualTrafficLightPanel::update_table(const std::vector<ReceivedCommand> & commands)
{
  std::vector<std::string> keys;
  keys.reserve(commands.size());
  for (const auto & command : commands) {
    keys.push_back(make_key(command.type, command.id));
  }

  if (keys != row_keys_) {
    table_->setRowCount(static_cast<int>(commands.size()));
    for (size_t i = 0; i < commands.size(); ++i) {
      const auto row = static_cast<int>(i);
      const auto & command = commands[i];
      const Answer & answer = answers_[keys[i]];
      table_->setCellWidget(row, COLUMN_ID, make_cell_label(QString::fromStdString(command.id)));
      table_->setCellWidget(row, COLUMN_STATE, make_cell_label(""));
      table_->setCellWidget(
        row, COLUMN_LANE, make_cell_label(QString::fromStdString(command.lane_id)));
      table_->setCellWidget(
        row, COLUMN_TYPE, make_cell_label(QString::fromStdString(command.type)));
      table_->setCellWidget(
        row, COLUMN_APPROVAL, make_answer_button(keys[i], false, answer.approval));
      table_->setCellWidget(
        row, COLUMN_FINALIZED, make_answer_button(keys[i], true, answer.is_finalized));
    }
    row_keys_ = keys;
  }
  keep_answer_columns_visible();

  for (size_t i = 0; i < commands.size(); ++i) {
    const auto row = static_cast<int>(i);
    cell_label(row, COLUMN_STATE)->setText(to_state_name(commands[i].state));
    const bool waiting = is_waiting_for_infrastructure(commands[i].state, answers_[keys[i]]);
    for (const int column : {COLUMN_ID, COLUMN_STATE, COLUMN_LANE, COLUMN_TYPE}) {
      set_cell_style(cell_label(row, column), waiting ? HIGHLIGHTED_CELL_STYLE : CELL_STYLE);
    }
  }

  if (!input_topic_error_.isEmpty() || !output_topic_error_.isEmpty()) {
    status_label_->setText((input_topic_error_ + " " + output_topic_error_).trimmed());
  } else if (commands.empty()) {
    status_label_->setText("No virtual traffic light command received.");
  } else if (!publish_button_->isChecked()) {
    status_label_->setText("Not publishing: planning receives no answer.");
  } else {
    status_label_->setText(QString("Publishing %1 state(s).").arg(commands.size()));
  }
}

QLabel * VirtualTrafficLightPanel::cell_label(const int row, const int column) const
{
  return qobject_cast<QLabel *>(table_->cellWidget(row, column));
}

void VirtualTrafficLightPanel::keep_answer_columns_visible()
{
  // Column widths are only valid after Qt has laid the table out, so this runs on every refresh.
  int width = 2 * table_->frameWidth() + table_->verticalScrollBar()->sizeHint().width();
  for (int column = COLUMN_ID; column <= COLUMN_FINALIZED; ++column) {
    width += table_->columnWidth(column);
  }
  if (width != table_->minimumWidth()) {
    table_->setMinimumWidth(width);
  }
}

void VirtualTrafficLightPanel::on_publish_toggled(const bool checked)
{
  style_publish_button(publish_button_, checked);
}

QPushButton * VirtualTrafficLightPanel::make_answer_button(
  const std::string & key, const bool is_finalization, const bool value)
{
  auto * button = new QPushButton();
  button->setCheckable(true);
  button->setChecked(value);
  button->setFocusPolicy(Qt::NoFocus);
  style_answer_button(button, value);
  connect(button, &QPushButton::toggled, this, [this, key, is_finalization, button](bool checked) {
    Answer & answer = answers_[key];
    if (is_finalization) {
      answer.is_finalized = checked;
    } else {
      answer.approval = checked;
    }
    style_answer_button(button, checked);
  });
  return button;
}

void VirtualTrafficLightPanel::publish_states(const std::vector<ReceivedCommand> & commands)
{
  if (!pub_states_ || !publish_button_->isChecked() || commands.empty()) {
    return;
  }

  VirtualTrafficLightStateArray msg;
  msg.stamp = raw_node_->now();
  for (const auto & command : commands) {
    const Answer & answer = answers_[make_key(command.type, command.id)];
    VirtualTrafficLightState state;
    state.stamp = msg.stamp;
    state.type = command.type;
    state.id = command.id;
    state.approval = answer.approval;
    state.is_finalized = answer.is_finalized;
    msg.states.push_back(std::move(state));
  }
  pub_states_->publish(msg);
}

void VirtualTrafficLightPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("input_topic", input_topic_edit_->text());
  config.mapSetValue("output_topic", output_topic_edit_->text());
}

void VirtualTrafficLightPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("input_topic", &topic)) {
    input_topic_edit_->setText(topic);
    create_subscription();
  }
  if (config.mapGetString("output_topic", &topic)) {
    output_topic_edit_->setText(topic);
    create_publisher();
  }
}

}  // namespace autoware::virtual_traffic_light_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::virtual_traffic_light_rviz_plugin::VirtualTrafficLightPanel, rviz_common::Panel)
