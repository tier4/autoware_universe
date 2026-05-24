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

#include "autoware/system_monitor_host/plugins/process_monitor_plugin.hpp"

#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <sstream>
#include <string>

namespace autoware::system_monitor_host::plugin
{

void ProcessMonitorPlugin::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<diagnostic_updater::Updater> & updater)
{
  node_ptr_ = node_ptr;
  updater_ = updater;
  name_ = name;

  gethostname(hostname_, sizeof(hostname_));

  setup_params();

  updater_->setHardwareID(hostname_);
  updater_->add("Tasks Summary", this, &ProcessMonitorPlugin::check_tasks_summary);

  timer_ = rclcpp::create_timer(
    node_ptr_, node_ptr_->get_clock(), std::chrono::seconds(1),
    std::bind(&ProcessMonitorPlugin::on_timer, this));
}

void ProcessMonitorPlugin::setup_params()
{
  node_ptr_->declare_parameter("process_monitor.num_of_procs", 5);
  num_of_procs_ = node_ptr_->get_parameter("process_monitor.num_of_procs").as_int();
}

rcl_interfaces::msg::SetParametersResult ProcessMonitorPlugin::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : parameters) {
    if (p.get_name() == "process_monitor.num_of_procs") num_of_procs_ = p.as_int();
  }
  return result;
}

void ProcessMonitorPlugin::evaluate()
{
  updater_->force_update();
}

void ProcessMonitorPlugin::on_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Read /proc/stat for task counts
  std::ifstream stat_file("/proc/stat");
  if (stat_file.is_open()) {
    std::string line;
    while (std::getline(stat_file, line)) {
      if (line.rfind("procs_running", 0) == 0) {
        running_tasks_ = std::stoi(line.substr(14));
      } else if (line.rfind("procs_blocked", 0) == 0) {
        // blocked
      }
    }
  }

  // Read /proc/loadavg for total tasks
  std::ifstream load_file("/proc/loadavg");
  if (load_file.is_open()) {
    std::string line;
    std::getline(load_file, line);
    std::istringstream iss(line);
    std::string token;
    iss >> token >> token >> token >> token;  // skip loadavg + running/total
    // format: "1.23 0.45 0.67 2/123 1234"
    std::string task_str;
    iss >> task_str;
    size_t slash = task_str.find('/');
    if (slash != std::string::npos) {
      running_tasks_ = std::stoi(task_str.substr(0, slash));
      total_tasks_ = std::stoi(task_str.substr(slash + 1));
    }
  }

  // Scan /proc for high CPU/Mem
  high_cpu_procs_.clear();
  high_mem_procs_.clear();

  DIR * dir = opendir("/proc");
  if (dir) {
    struct dirent * entry;
    while ((entry = readdir(dir)) != nullptr) {
      if (entry->d_type != DT_DIR) continue;
      int pid = std::atoi(entry->d_name);
      if (pid <= 0) continue;

      ProcInfo info;
      info.pid = pid;

      std::string stat_path = "/proc/" + std::to_string(pid) + "/stat";
      std::ifstream stat(stat_path);
      if (stat.is_open()) {
        std::string line;
        std::getline(stat, line);
        // Parse comm (enclosed in parens)
        size_t open_paren = line.find('(');
        size_t close_paren = line.rfind(')');
        if (open_paren != std::string::npos && close_paren != std::string::npos) {
          info.command = line.substr(open_paren + 1, close_paren - open_paren - 1);

          // Get state after closing paren
          std::istringstream rest(line.substr(close_paren + 2));
          char state;
          rest >> state;

          if (state == 'Z') zombie_tasks_++;
          else if (state == 'T') stopped_tasks_++;
          else if (state == 'S' || state == 'I') sleeping_tasks_++;
          else if (state == 'R') running_tasks_++;
        }
      }

      // Get memory from /proc/[pid]/statm
      std::string statm_path = "/proc/" + std::to_string(pid) + "/statm";
      std::ifstream statm(statm_path);
      if (statm.is_open()) {
        long size;
        statm >> size;
        info.mem_pct = static_cast<float>(size) * 4.0f / 1024.0f / 1024.0f;  // pages to GB approx
        high_mem_procs_.push_back(info);
      }
    }
    closedir(dir);
  }

  // Sort and trim
  std::sort(high_mem_procs_.begin(), high_mem_procs_.end(),
    [](const ProcInfo & a, const ProcInfo & b) { return a.mem_pct > b.mem_pct; });
  if (static_cast<int>(high_mem_procs_.size()) > num_of_procs_) {
    high_mem_procs_.resize(num_of_procs_);
  }
}

void ProcessMonitorPlugin::check_tasks_summary(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  int level = DiagStatus::OK;
  std::string msg = "OK";

  stat.add("total", total_tasks_);
  stat.add("running", running_tasks_);
  stat.add("sleeping", sleeping_tasks_);
  stat.add("stopped", stopped_tasks_);
  stat.add("zombie", zombie_tasks_);

  if (zombie_tasks_ > 0) {
    level = DiagStatus::WARN;
    msg = "zombie processes found";
  }

  for (size_t i = 0; i < high_mem_procs_.size(); ++i) {
    std::string key = "High-mem Proc[" + std::to_string(i) + "]";
    stat.add(key + ": PID", high_mem_procs_[i].pid);
    stat.add(key + ": command", high_mem_procs_[i].command);
    stat.add(key + ": memory_usage", high_mem_procs_[i].mem_pct);
  }

  stat.summary(level, msg);
}

}  // namespace autoware::system_monitor_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::system_monitor_host::plugin::ProcessMonitorPlugin,
  autoware::system_monitor_host::plugin::MonitorPluginBase)
