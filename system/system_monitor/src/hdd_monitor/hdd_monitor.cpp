// Copyright 2020 Autoware Foundation
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

/**
 * @file hdd_monitor.cpp
 * @brief HDD monitor class
 */

#include "system_monitor/hdd_monitor/hdd_monitor.hpp"

#include "system_monitor/system_monitor_utility.hpp"

#include <hdd_reader/hdd_reader.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/process.hpp>
#include <boost/serialization/vector.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <string>
#include <vector>

namespace bp = boost::process;

HDDMonitor::HDDMonitor(const rclcpp::NodeOptions & options)
: Node("hdd_monitor", options),
  updater_(this),
  hdd_reader_port_(declare_parameter<int>("hdd_reader_port", 7635))
{
  gethostname(hostname_, sizeof(hostname_));

  getHDDParams();

  updater_.setHardwareID(hostname_);
  updater_.add("HDD Temperature", this, &HDDMonitor::checkSMARTTemperature);
  updater_.add("HDD PowerOnHours", this, &HDDMonitor::checkSMARTPowerOnHours);
  updater_.add("HDD TotalDataWritten", this, &HDDMonitor::checkSMARTTotalDataWritten);
  updater_.add("HDD Usage", this, &HDDMonitor::checkUsage);

  for (uint32_t i = 0; i < static_cast<uint32_t>(HDDSMARTInfoItem::SIZE); i++) {
    is_up_to_date_[i] = false;
  }
}

void HDDMonitor::update() { updater_.force_update(); }

void HDDMonitor::checkSMARTTemperature(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkSMART(stat, HDDSMARTInfoItem::TEMPERATURE);
}

void HDDMonitor::checkSMARTPowerOnHours(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkSMART(stat, HDDSMARTInfoItem::POWER_ON_HOURS);
}

void HDDMonitor::checkSMARTTotalDataWritten(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkSMART(stat, HDDSMARTInfoItem::TOTAL_DATA_WRITTEN);
}

void HDDMonitor::checkSMART(
  diagnostic_updater::DiagnosticStatusWrapper & stat, HDDSMARTInfoItem item)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (hdd_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  // Update HDD information
  if (!updateHDDInfoList(item, stat)) {
    return;
  }

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;
  std::string error_str = "";

  for (auto itr = hdd_params_.begin(); itr != hdd_params_.end(); ++itr, ++index) {
    // Retrieve HDD information
    auto hdd_itr = hdd_info_list_.find(itr->second.device_);
    if (hdd_itr == hdd_info_list_.end()) {
      stat.add(fmt::format("HDD {}: status", index), "hdd_reader error");
      stat.add(fmt::format("HDD {}: name", index), itr->first.c_str());
      stat.add(fmt::format("HDD {}: hdd_reader", index), strerror(ENOENT));
      error_str = "hdd_reader error";
      continue;
    }

    if (hdd_itr->second.error_code_ != 0) {
      stat.add(fmt::format("HDD {}: status", index), "hdd_reader error");
      stat.add(fmt::format("HDD {}: name", index), itr->first.c_str());
      stat.add(fmt::format("HDD {}: hdd_reader", index), strerror(hdd_itr->second.error_code_));
      error_str = "hdd_reader error";
      continue;
    }

    stat.add(
      fmt::format("HDD {}: status", index), smart_dicts_[static_cast<uint32_t>(item)].at(level));
    stat.add(fmt::format("HDD {}: name", index), itr->second.device_.c_str());
    stat.add(fmt::format("HDD {}: model", index), hdd_itr->second.model_.c_str());
    stat.add(fmt::format("HDD {}: serial", index), hdd_itr->second.serial_.c_str());
    switch (item) {
      case HDDSMARTInfoItem::TEMPERATURE: {
        float temp = static_cast<float>(hdd_itr->second.temp_);

        level = DiagStatus::OK;
        if (temp >= itr->second.temp_error_) {
          level = DiagStatus::ERROR;
        } else if (temp >= itr->second.temp_warn_) {
          level = DiagStatus::WARN;
        }
        stat.addf(fmt::format("HDD {}: temperature", index), "%.1f DegC", temp);
      } break;
      case HDDSMARTInfoItem::POWER_ON_HOURS: {
        int64_t power_on_hours = static_cast<int64_t>(hdd_itr->second.power_on_hours_);

        level = DiagStatus::OK;
        if (power_on_hours >= itr->second.power_on_hours_error_) {
          level = DiagStatus::ERROR;
        } else if (power_on_hours >= itr->second.power_on_hours_warn_) {
          level = DiagStatus::WARN;
        }
        stat.addf(
          fmt::format("HDD {}: power on hours", index), "%u Hours",
          hdd_itr->second.power_on_hours_);
      } break;
      case HDDSMARTInfoItem::TOTAL_DATA_WRITTEN: {
        int64_t total_data_written = static_cast<int64_t>(hdd_itr->second.total_data_written_);

        level = DiagStatus::OK;
        if (total_data_written >= itr->second.total_data_written_error_) {
          level = DiagStatus::ERROR;
        } else if (total_data_written >= itr->second.total_data_written_warn_) {
          level = DiagStatus::WARN;
        }
        stat.addf(
          fmt::format("HDD {}: total written", index), "%u", hdd_itr->second.total_data_written_);
      } break;
      default:
        break;
    }

    whole_level = std::max(whole_level, level);
  }

  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, error_str);
  } else {
    stat.summary(whole_level, smart_dicts_[static_cast<uint32_t>(item)].at(whole_level));
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void HDDMonitor::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (hdd_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  int hdd_index = 0;
  int whole_level = DiagStatus::OK;
  std::string error_str = "";

  for (auto itr = hdd_params_.begin(); itr != hdd_params_.end(); ++itr, ++hdd_index) {
    // Get summary of disk space usage of ext4
    bp::ipstream is_out;
    bp::ipstream is_err;
    // Invoke shell to use shell wildcard expansion
    bp::child c(
      "/bin/sh", "-c", fmt::format("df -Pm {}*", itr->first.c_str()), bp::std_out > is_out,
      bp::std_err > is_err);
    c.wait();

    if (c.exit_code() != 0) {
      std::ostringstream os;
      is_err >> os.rdbuf();
      error_str = "df error";
      stat.add(fmt::format("HDD {}: status", hdd_index), "df error");
      stat.add(fmt::format("HDD {}: name", hdd_index), itr->first.c_str());
      stat.add(fmt::format("HDD {}: df", hdd_index), os.str().c_str());
      continue;
    }

    int level = DiagStatus::OK;
    std::string line;
    int index = 0;
    std::vector<std::string> list;
    int avail;

    while (std::getline(is_out, line) && !line.empty()) {
      // Skip header
      if (index <= 0) {
        ++index;
        continue;
      }

      boost::split(list, line, boost::is_space(), boost::token_compress_on);

      try {
        avail = std::stoi(list[3].c_str());
      } catch (std::exception & e) {
        avail = -1;
        error_str = e.what();
        stat.add(fmt::format("HDD {}: status", hdd_index), "avail string error");
      }

      if (avail <= itr->second.free_error_) {
        level = DiagStatus::ERROR;
      } else if (avail <= itr->second.free_warn_) {
        level = DiagStatus::WARN;
      } else {
        level = DiagStatus::OK;
      }

      stat.add(fmt::format("HDD {}: status", hdd_index), usage_dict_.at(level));
      stat.add(fmt::format("HDD {}: filesystem", hdd_index), list[0].c_str());
      stat.add(fmt::format("HDD {}: size", hdd_index), (list[1] + " MiB").c_str());
      stat.add(fmt::format("HDD {}: used", hdd_index), (list[2] + " MiB").c_str());
      stat.add(fmt::format("HDD {}: avail", hdd_index), (list[3] + " MiB").c_str());
      stat.add(fmt::format("HDD {}: use", hdd_index), list[4].c_str());
      std::string mounted_ = list[5];
      if (list.size() > 6) {
        std::string::size_type pos = line.find("% /");
        if (pos != std::string::npos) {
          mounted_ = line.substr(pos + 2);  // 2 is "% " length
        }
      }
      stat.add(fmt::format("HDD {}: mounted on", hdd_index), mounted_.c_str());

      whole_level = std::max(whole_level, level);
      ++index;
    }
  }

  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, error_str);
  } else {
    stat.summary(whole_level, usage_dict_.at(whole_level));
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void HDDMonitor::getHDDParams()
{
  const auto num_disks = this->declare_parameter("num_disks", 0);
  for (auto i = 0; i < num_disks; ++i) {
    const auto prefix = "disks.disk" + std::to_string(i);
    const auto name = declare_parameter<std::string>(prefix + ".name");

    // Get device name from mount point
    const auto device_name = getDeviceFromMountPoint(name);
    if (device_name.empty()) {
      continue;
    }

    HDDParam param;
    param.temp_warn_ = declare_parameter<float>(prefix + ".temp_warn");
    param.temp_error_ = declare_parameter<float>(prefix + ".temp_error");
    param.power_on_hours_warn_ = declare_parameter<int>(prefix + ".power_on_hours_warn");
    param.power_on_hours_error_ = declare_parameter<int>(prefix + ".power_on_hours_error");
    param.total_data_written_safety_factor_ =
      declare_parameter<float>(prefix + ".total_data_written_safety_factor");
    int total_data_written_warn_org = declare_parameter<int>(prefix + ".total_data_written_warn");
    param.total_data_written_warn_ =
      static_cast<int>(total_data_written_warn_org * (1.0f - param.total_data_written_safety_factor_));
    int total_data_written_error_org = declare_parameter<int>(prefix + ".total_data_written_error");
    param.total_data_written_error_ =
      static_cast<int>(total_data_written_error_org * (1.0f - param.total_data_written_safety_factor_));
    param.free_warn_ = declare_parameter<int>(prefix + ".free_warn");
    param.free_error_ = declare_parameter<int>(prefix + ".free_error");

    // Remove index number of partition for passing device name to hdd-reader
    if (boost::starts_with(device_name, "/dev/sd")) {
      const std::regex pattern("\\d+$");
      param.device_ = std::regex_replace(device_name, pattern, "");
    } else {
      const std::regex pattern("(\\d+).*");
      param.device_ = std::regex_replace(device_name, pattern, "$1");
    }
    hdd_params_[device_name] = param;

    hdd_devices_.push_back(param.device_);
  }
}

std::string HDDMonitor::getDeviceFromMountPoint(const std::string & mount_point)
{
  std::string ret;
  bp::ipstream is_out;
  bp::ipstream is_err;

  bp::child c(
    "/bin/sh", "-c", fmt::format("findmnt -n -o SOURCE {}", mount_point.c_str()),
    bp::std_out > is_out, bp::std_err > is_err);
  c.wait();

  if (c.exit_code() != 0) {
    RCLCPP_ERROR(get_logger(), "Failed to execute findmnt. %s", mount_point.c_str());
    return "";
  }

  if (!std::getline(is_out, ret)) {
    RCLCPP_ERROR(get_logger(), "Failed to find device name. %s", mount_point.c_str());
    return "";
  }

  return ret;
}

bool HDDMonitor::updateHDDInfoList(
  HDDSMARTInfoItem item, diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  static uint32_t size = static_cast<uint32_t>(HDDSMARTInfoItem::SIZE);
  uint32_t item_index = static_cast<uint32_t>(item);
  if (item_index >= size) {
    stat.summary(DiagStatus::ERROR, "invalid S.M.A.R.T. information");
    return false;
  }

  // Connect to HDD reader if the item is out of date
  if (!is_up_to_date_[item_index]) {
    if (!getHDDInfoListFromHDDReader(stat)) {
      return false;
    }
    // Set flags of all item to up to date
    for (uint32_t i = 0; i < size; i++) {
      is_up_to_date_[i] = true;
    }
  }

  // Set flag of the item to out of date
  is_up_to_date_[item_index] = false;
  return true;
}

bool HDDMonitor::getHDDInfoListFromHDDReader(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Create a new socket
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    stat.summary(DiagStatus::ERROR, "socket error");
    stat.add("socket", strerror(errno));
    return false;
  }

  // Specify the receiving timeouts until reporting an error
  struct timeval tv;
  tv.tv_sec = 10;
  tv.tv_usec = 0;
  int ret = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "setsockopt error");
    stat.add("setsockopt", strerror(errno));
    close(sock);
    return false;
  }

  // Connect the socket referred to by the file descriptor
  sockaddr_in addr;
  memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(hdd_reader_port_);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  ret = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "connect error");
    stat.add("connect", strerror(errno));
    close(sock);
    return false;
  }

  std::ostringstream oss;
  boost::archive::text_oarchive oa(oss);
  oa & hdd_devices_;

  // Write list of devices to FD
  ret = write(sock, oss.str().c_str(), oss.str().length());
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "write error");
    stat.add("write", strerror(errno));
    RCLCPP_ERROR(get_logger(), "write error");
    close(sock);
    return false;
  }

  // Receive messages from a socket
  char buf[1024] = "";
  ret = recv(sock, buf, sizeof(buf) - 1, 0);
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "recv error");
    stat.add("recv", strerror(errno));
    close(sock);
    return false;
  }
  // No data received
  if (ret == 0) {
    stat.summary(DiagStatus::ERROR, "recv error");
    stat.add("recv", "No data received");
    close(sock);
    return false;
  }

  // Close the file descriptor FD
  ret = close(sock);
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "close error");
    stat.add("close", strerror(errno));
    return false;
  }

  // Restore HDD information list
  try {
    std::istringstream iss(buf);
    boost::archive::text_iarchive oa(iss);
    oa >> hdd_info_list_;
  } catch (const std::exception & e) {
    stat.summary(DiagStatus::ERROR, "recv error");
    stat.add("recv", e.what());
    return false;
  }

  return true;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(HDDMonitor)
