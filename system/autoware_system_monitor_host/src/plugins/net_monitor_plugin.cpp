// Copyright 2025 TIER IV, Inc.
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

#include "autoware/system_monitor_host/plugins/net_monitor_plugin.hpp"

#include <ifaddrs.h>
#include <net/if.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

namespace autoware::system_monitor_host::plugin
{

void NetMonitorPlugin::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<diagnostic_updater::Updater> & updater)
{
  node_ptr_ = node_ptr;
  updater_ = updater;
  name_ = name;

  gethostname(hostname_, sizeof(hostname_));

  pub_net_ = node_ptr_->create_publisher<tier4_external_api_msgs::msg::NetworkStatus>(
    "~/network_status", rclcpp::QoS(1).transient_local());

  setup_params();

  updater_->setHardwareID(hostname_);
  updater_->add("Network Connection", this, &NetMonitorPlugin::check_connection);
  updater_->add("Network Usage", this, &NetMonitorPlugin::check_usage);
  updater_->add("IP Packet Reassembles Failed", this, &NetMonitorPlugin::check_ip_packet_reassembles);
  updater_->add("UDP Buf Errors", this, &NetMonitorPlugin::check_udp_buf_errors);

  timer_ = rclcpp::create_timer(
    node_ptr_, node_ptr_->get_clock(), std::chrono::seconds(1),
    std::bind(&NetMonitorPlugin::on_timer, this));
}

void NetMonitorPlugin::setup_params()
{
  node_ptr_->declare_parameter("net_monitor.devices", std::vector<std::string>({"eth0", "wlan0"}));
  devices_ = node_ptr_->get_parameter("net_monitor.devices").as_string_array();
}

rcl_interfaces::msg::SetParametersResult NetMonitorPlugin::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : parameters) {
    if (p.get_name() == "net_monitor.devices") devices_ = p.as_string_array();
  }
  return result;
}

void NetMonitorPlugin::evaluate()
{
  updater_->force_update();
}

void NetMonitorPlugin::on_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);

  all_devices_found_ = true;
  struct ifaddrs * ifaddr = nullptr;
  if (getifaddrs(&ifaddr) == 0) {
    for (const auto & dev : devices_) {
      bool found = false;
      for (auto * ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_name && dev == ifa->ifa_name) {
          found = true;
          break;
        }
      }
      if (!found) all_devices_found_ = false;
    }
    freeifaddrs(ifaddr);
  }

  tier4_external_api_msgs::msg::NetworkStatus net_msg;
  net_msg.stamp = node_ptr_->now();
  net_msg.hostname = hostname_;
  for (const auto & dev : devices_) {
    tier4_external_api_msgs::msg::NetworkInterfaceStatus iface;
    iface.name = dev;
    iface.rx_traffic = 0.0f;
    iface.tx_traffic = 0.0f;
    iface.capacity = 0.0f;
    iface.rx_errors = 0;
    iface.tx_errors = 0;
    iface.collisions = 0;
    net_msg.interfaces.push_back(iface);
  }
  pub_net_->publish(net_msg);
}

void NetMonitorPlugin::check_connection(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  int level = DiagStatus::OK;
  if (!all_devices_found_) {
    level = DiagStatus::ERROR;
    stat.add("Missing devices", "some configured devices not found");
  }
  for (const auto & dev : devices_) {
    stat.add(dev, all_devices_found_ ? "up" : "missing");
  }
  stat.summary(level, level == DiagStatus::OK ? "OK" : "device missing");
}

void NetMonitorPlugin::check_usage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.summary(DiagStatus::OK, "OK");
}

void NetMonitorPlugin::check_ip_packet_reassembles(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  int level = DiagStatus::OK;
  std::ifstream snmp("/proc/net/snmp");
  if (snmp.is_open()) {
    std::string header, values;
    std::getline(snmp, header);
    std::getline(snmp, values);
    if (header.find("Ip:") == 0) {
      std::istringstream hdr(header);
      std::istringstream val(values);
      std::string vh, vv;
      hdr >> vh; val >> vv;
      while (hdr >> vh && val >> vv) {
        if (vh == "ReasmFails") {
          long fails = std::stol(vv);
          stat.add("reasm_fails", fails);
          if (fails > 0) level = DiagStatus::WARN;
          break;
        }
      }
    }
  }
  stat.summary(level, level == DiagStatus::OK ? "OK" : "reassembly failures detected");
}

void NetMonitorPlugin::check_udp_buf_errors(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  int level = DiagStatus::OK;
  std::ifstream snmp("/proc/net/snmp");
  if (snmp.is_open()) {
    std::string line;
    std::string header, values;
    std::getline(snmp, header); std::getline(snmp, values);  // Ip
    std::getline(snmp, header); std::getline(snmp, values);  // Icmp
    std::getline(snmp, header); std::getline(snmp, values);  // Tcp
    std::getline(snmp, header); std::getline(snmp, values);  // Udp
    if (header.find("Udp:") == 0) {
      std::istringstream hdr(header);
      std::istringstream val(values);
      std::string vh, vv;
      hdr >> vh; val >> vv;
      while (hdr >> vh && val >> vv) {
        if (vh == "RcvbufErrors" || vh == "SndbufErrors") {
          stat.add(vh, std::stol(vv));
        }
      }
    }
  }
  stat.summary(level, "OK");
}

}  // namespace autoware::system_monitor_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::system_monitor_host::plugin::NetMonitorPlugin,
  autoware::system_monitor_host::plugin::MonitorPluginBase)
