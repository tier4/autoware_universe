// Copyright 2020,2025 Tier IV, Inc.
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

#include "system_monitor/cpu_monitor/arm_cpu_monitor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/process.hpp>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
constexpr const char * TEST_FILE = "test";
char ** argv_;
}  // namespace

class TestCPUMonitor : public CPUMonitor
{
  friend class CPUMonitorTestSuite;

public:
  TestCPUMonitor(const std::string & node_name, const rclcpp::NodeOptions & options)
  : CPUMonitor(node_name, options)
  {
  }

  void diagCallback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diag_msg)
  {
    std::lock_guard<std::mutex> lock_diagnostic(mutex_diagnostic_);
    array_ = *diag_msg;
  }

  void addTempName(const std::string & label, const std::string & path)
  {
    std::lock_guard<std::mutex> lock_context(mutex_context_);
    temperatures_.emplace_back(label, path);
  }

  void clearTempNames()
  {
    std::lock_guard<std::mutex> lock_context(mutex_context_);
    temperatures_.clear();
  }

  void addFreqName(int index, const std::string & path)
  {
    std::lock_guard<std::mutex> lock_context(mutex_context_);
    frequencies_.emplace_back(index, path);
  }

  void clearFreqNames()
  {
    std::lock_guard<std::mutex> lock_context(mutex_context_);
    frequencies_.clear();
  }

  void changeUsageWarn(float usage_warn)
  {
    std::lock_guard<std::mutex> lock_context(mutex_context_);
    usage_warn_ = usage_warn;
  }

  void changeUsageError(float usage_error)
  {
    std::lock_guard<std::mutex> lock_context(mutex_context_);
    usage_error_ = usage_error;
  }

// Warning/Error about CPU load average used to be implemented,
// but they were removed to avoid false alarms.
#ifdef ENABLE_LOAD_AVERAGE_DIAGNOSTICS
  void changeLoad1Warn(float load1_warn)
  {
    std::lock_guard<std::mutex> lock_context(mutex_context_);
    load1_warn_ = load1_warn;
  }

  void changeLoad5Warn(float load5_warn)
  {
    std::lock_guard<std::mutex> lock_context(mutex_context_);
    load5_warn_ = load5_warn;
  }
#endif  // ENABLE_LOAD_AVERAGE_DIAGNOSTICS

  void update() { updater_.force_update(); }

  void forceTimerEvent() { this->onTimer(); }

  void disableTimer() { timer_->cancel(); }

  const std::string removePrefix(const std::string & name)
  {
    return boost::algorithm::erase_all_copy(name, prefix_);
  }

  bool findDiagStatus(const std::string & name, DiagStatus & status)  // NOLINT
  {
    std::lock_guard<std::mutex> lock_diagnostic(mutex_diagnostic_);
    for (size_t i = 0; i < array_.status.size(); ++i) {
      if (removePrefix(array_.status[i].name) == name) {
        status = array_.status[i];
        return true;
      }
    }
    return false;
  }

private:
  std::mutex mutex_diagnostic_;  // Protects the diagnostic array.
  diagnostic_msgs::msg::DiagnosticArray array_;

  const std::string prefix_ = std::string(this->get_name()) + ": ";
};

class CPUMonitorTestSuite : public ::testing::Test
{
public:
  CPUMonitorTestSuite() : monitor_(nullptr), sub_(nullptr) {}

protected:
  std::unique_ptr<TestCPUMonitor> monitor_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;

  void SetUp()
  {
    using std::placeholders::_1;
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;
    monitor_ = std::make_unique<TestCPUMonitor>("test_cpu_monitor", node_options);
    // If the timer is enabled, it will interfere with the test.
    // NOTE:
    //   Though disabling the timer is necessary for the test,
    //   it makes the test run in a single thread context,
    //   different from the real case.
    monitor_->disableTimer();

    // The queue size is set to 1 so that the result of the changes made by the tests
    // can be delivered to the topic subscriber immediately.
    sub_ = monitor_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 1, std::bind(&TestCPUMonitor::diagCallback, monitor_.get(), _1));

    // error_code is used to avoid exceptions.
    std::error_code error_code;
    // Remove test file if exists
    if (fs::exists(TEST_FILE, error_code)) {
      fs::remove(TEST_FILE, error_code);
    }
  }

  void TearDown()
  {
    // error_code is used to avoid exceptions.
    std::error_code error_code;
    // Remove test file if exists
    if (fs::exists(TEST_FILE, error_code)) {
      fs::remove(TEST_FILE, error_code);
    }
    rclcpp::shutdown();
  }

  bool findValue(const DiagStatus status, const std::string & key, std::string & value)  // NOLINT
  {
    for (auto itr = status.values.begin(); itr != status.values.end(); ++itr) {
      if (itr->key == key) {
        value = itr->value;
        return true;
      }
    }
    return false;
  }

  void updatePublishSubscribe()
  {
    monitor_->forceTimerEvent();
    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());
  }
};

// Warning/Error about temperature used to be implemented,
// but they were removed in favor of warning/error about thermal throttling.
#ifdef ENABLE_TEMPERATURE_DIAGNOSTICS
TEST_F(CPUMonitorTestSuite, tempWarnTest)
{
  // Verify normal behavior
  {
    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Add test file to list
  monitor_->addTempName("CPU Dummy", TEST_FILE);

  // Verify warning
  {
    // Write warning level
    std::ofstream ofs(TEST_FILE);
    ofs << 90000 << std::endl;

    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Write normal level
    std::ofstream ofs(TEST_FILE);
    ofs << 89900 << std::endl;

    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(CPUMonitorTestSuite, tempErrorTest)
{
  // Verify normal behavior
  {
    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Add test file to list
  monitor_->addTempName("CPU Dummy", TEST_FILE);

  // Verify error
  {
    // Write error level
    std::ofstream ofs(TEST_FILE);
    ofs << 95000 << std::endl;

    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::ERROR);
  }

  // Verify normal behavior
  {
    // Write normal level
    std::ofstream ofs(TEST_FILE);
    ofs << 89900 << std::endl;

    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}
#endif  // ENABLE_TEMPERATURE_DIAGNOSTICS

TEST_F(CPUMonitorTestSuite, tempTemperatureFilesNotFoundTest)
{
  // Make it sure that lazy initialization is done.
  monitor_->forceTimerEvent();

  // Clear list
  monitor_->clearTempNames();

  updatePublishSubscribe();

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("CPU Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "temperature files not found");
}

TEST_F(CPUMonitorTestSuite, tempFileOpenErrorTest)
{
  // Make it sure that lazy initialization is done.
  monitor_->forceTimerEvent();

  // Add test file to list
  monitor_->addTempName("CPU Dummy", TEST_FILE);

  updatePublishSubscribe();

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("CPU Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "file open error");
  ASSERT_TRUE(findValue(status, "file open error", value));
  ASSERT_STREQ(value.c_str(), TEST_FILE);
}

TEST_F(CPUMonitorTestSuite, usageWarnTest)
{
  // Verify normal behavior
  {
    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    std::string value;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeUsageWarn(0.0);

    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeUsageWarn(0.90);

    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(CPUMonitorTestSuite, usageErrorTest)
{
  // Verify normal behavior
  {
    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    std::string value;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify error
  {
    // Change warning level
    monitor_->changeUsageError(0.0);

    updatePublishSubscribe();

    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Usage", status));
    // It requires consecutive two errors to set ERROR.
    ASSERT_EQ(status.level, DiagStatus::OK);

    updatePublishSubscribe();

    ASSERT_TRUE(monitor_->findDiagStatus("CPU Usage", status));
    // This time, ERROR should be reported.
    ASSERT_EQ(status.level, DiagStatus::ERROR);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeUsageError(1.00);

    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

// Warning/Error about CPU load average used to be implemented,
// but they were removed to avoid false alarms.
#ifdef ENABLE_LOAD_AVERAGE_DIAGNOSTICS
TEST_F(CPUMonitorTestSuite, load1WarnTest)
{
  // Verify normal behavior
  {
    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    std::string value;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Load Average", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeLoad1Warn(0.0);

    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Load Average", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeLoad1Warn(0.90);

    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Load Average", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(CPUMonitorTestSuite, load5WarnTest)
{
  // Verify normal behavior
  {
    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    std::string value;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Load Average", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeLoad5Warn(0.0);

    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Load Average", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeLoad5Warn(0.80);

    updatePublishSubscribe();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Load Average", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}
#endif  // ENABLE_LOAD_AVERAGE_DIAGNOSTICS

TEST_F(CPUMonitorTestSuite, DISABLED_throttlingTest)
{
  updatePublishSubscribe();

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("CPU Thermal Throttling", status));
  ASSERT_EQ(status.level, DiagStatus::OK);
}

TEST_F(CPUMonitorTestSuite, freqTest)
{
  updatePublishSubscribe();

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("CPU Frequency", status));
  ASSERT_EQ(status.level, DiagStatus::OK);
}

TEST_F(CPUMonitorTestSuite, freqFrequencyFilesNotFoundTest)
{
  // Make it sure that lazy initialization is done.
  monitor_->forceTimerEvent();

  // Clear list
  monitor_->clearFreqNames();

  updatePublishSubscribe();

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("CPU Frequency", status));

  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "frequency files not found");
}

// for coverage
class DummyCPUMonitor : public CPUMonitorBase
{
  friend class CPUMonitorTestSuite;

public:
  DummyCPUMonitor(const std::string & node_name, const rclcpp::NodeOptions & options)
  : CPUMonitorBase(node_name, options)
  {
  }
  void update() { updater_.force_update(); }
  void forceTimerEvent() { this->onTimer(); }
};

TEST_F(CPUMonitorTestSuite, dummyCPUMonitorTest)
{
  rclcpp::NodeOptions options;
  std::unique_ptr<DummyCPUMonitor> monitor =
    std::make_unique<DummyCPUMonitor>("dummy_cpu_monitor", options);
  monitor->forceTimerEvent();
  // Publish topic
  monitor->update();
}

int main(int argc, char ** argv)
{
  argv_ = argv;
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
