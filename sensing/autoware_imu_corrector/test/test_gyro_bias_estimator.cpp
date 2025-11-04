// Copyright 2025 Tier IV, Inc.
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

#include "../src/gyro_bias_estimator.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <array>
#include <random>

class GyroBiasEstimationModuleTest : public autoware::imu_corrector::GyroBiasEstimationModule
{
public:
  using GyroBiasEstimationModule::GyroBiasEstimationModule;
  int get_buffer_size() const { return gyro_buffer_.size(); }
  bool get_is_buffer_full() const { return is_gyro_buffer_full_; }
  bool get_can_calibrate() const { return can_calibrate_; }
  void set_gyro_buffer(boost::circular_buffer<geometry_msgs::msg::Vector3> & buffer)
  {
    gyro_buffer_ = buffer;
  }

  int64_t get_min_duration_ms() const { return min_duration_ms_; }
  int64_t get_max_duration_ms() const { return max_duration_ms_; }
  std::optional<std::chrono::system_clock::time_point> last_update_time_;
  int64_t min_duration_ms_ = std::numeric_limits<int64_t>::max();
  int64_t max_duration_ms_ = 0;
};

class GyroBiasEstimationModuleTestForCheckRate : public GyroBiasEstimationModuleTest
{
public:
  using GyroBiasEstimationModuleTest::GyroBiasEstimationModuleTest;

  void update_gyro_buffer_full_flag(
    boost::circular_buffer<geometry_msgs::msg::Vector3> & buffer) override
  {
    auto current_time = std::chrono::system_clock::now();
    if (last_update_time_) {
      auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(current_time - *last_update_time_)
          .count();

      min_duration_ms_ = std::min(min_duration_ms_, duration);
      max_duration_ms_ = std::max(max_duration_ms_, duration);

      RCLCPP_INFO(
        rclcpp::get_logger("GyroBiasEstimationModuleTest"),
        "Time since last update: %ld ms (min: %ld ms, max: %ld ms)", duration, min_duration_ms_,
        max_duration_ms_);
    }
    last_update_time_ = current_time;

    is_gyro_buffer_full_ = buffer.full();
    RCLCPP_INFO(
      rclcpp::get_logger("GyroBiasEstimationModuleTest"), "is_gyro_buffer_full_: %d",
      is_gyro_buffer_full_);
  }
};

class GyroBiasEstimatorTest : public autoware::imu_corrector::GyroBiasEstimator
{
public:
  explicit GyroBiasEstimatorTest(
    double stddev_threshold = 0.000140, bool test_check_buffer_full_rate = false)
  : GyroBiasEstimator(setupNodeOptions(stddev_threshold))
  {
    const double timestamp_threshold = get_parameter("timestamp_threshold").as_double();
    const size_t data_num_threshold = get_parameter("data_num_threshold").as_int();
    const double bias_change_threshold = get_parameter("bias_change_threshold").as_double();
    if (test_check_buffer_full_rate) {
      gyro_bias_estimation_module_ = std::make_unique<GyroBiasEstimationModuleTestForCheckRate>(
        timestamp_threshold, data_num_threshold, bias_change_threshold, stddev_threshold,
        get_logger(), get_clock());
    } else {
      gyro_bias_estimation_module_ = std::make_unique<GyroBiasEstimationModuleTest>(
        timestamp_threshold, data_num_threshold, bias_change_threshold, stddev_threshold,
        get_logger(), get_clock());
    }
  }

  // on_timer()をpublicにアクセスできるようにするメソッドを追加
  void test_on_timer() { on_timer(); }

  GyroBiasEstimationModuleTest * get_estimation_module()
  {
    return static_cast<GyroBiasEstimationModuleTest *>(gyro_bias_estimation_module_.get());
  }

private:
  static rclcpp::NodeOptions setupNodeOptions(double stddev_threshold)
  {
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides().push_back(rclcpp::Parameter("timestamp_threshold", 0.1));
    node_options.parameter_overrides().push_back(rclcpp::Parameter("data_num_threshold", 400));
    node_options.parameter_overrides().push_back(
      rclcpp::Parameter("bias_change_threshold", 0.00061));
    node_options.parameter_overrides().push_back(
      rclcpp::Parameter("stddev_threshold", stddev_threshold));
    return node_options;
  }
};

// Helper function to publish static TF
void publishStaticTransform(const rclcpp::Node::SharedPtr & node)
{
  auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node->now();
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "imu_link";
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(transform);
}

std::array<double, 400> GenerateTestData(
  unsigned int seed = 42, double mean = 0.0, double stddev = 0.0001)
{
  std::array<double, 400> data;
  std::mt19937 gen(seed);
  std::normal_distribution<> dist(mean, stddev);

  for (size_t i = 0; i < data.size(); ++i) {
    data[i] = dist(gen);
  }
  return data;
}

double CalculateMedian(const std::array<double, 400> & data)
{
  std::array<double, 400> sorted_data = data;
  std::sort(sorted_data.begin(), sorted_data.end());

  const size_t mid = sorted_data.size() / 2;
  return (sorted_data[mid - 1] + sorted_data[mid]) / 2.0;
}

// Test fixture to initialize ROS context once for all tests
class GyroBiasEstimatorTestFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

// DT_1_3
TEST_F(GyroBiasEstimatorTestFixture, DT_1_3_1)
{
  sensor_msgs::msg::Imu imu;
  geometry_msgs::msg::TwistWithCovarianceStamped twist;
  auto node = std::make_shared<GyroBiasEstimatorTest>(0.000104, false);
  auto test_node = rclcpp::Node::make_shared("test_node");

  // Publish static TF
  publishStaticTransform(test_node);

  auto imu_pub = test_node->create_publisher<sensor_msgs::msg::Imu>(
    "/gyro_bias_validator/input/imu_raw", rclcpp::SensorDataQoS());
  auto twist_pub = test_node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/gyro_bias_validator/input/twist", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);

  imu.header.stamp = rclcpp::Clock().now();
  imu.header.frame_id = "imu_link";
  imu.angular_velocity.x = 0.0;
  imu.angular_velocity.y = 0.0;
  imu.angular_velocity.z = 0.0;

  twist.header.stamp = rclcpp::Clock().now();
  twist.header.frame_id = "base_link";
  twist.twist.twist.linear.x = 0.0;
  twist.twist.twist.linear.y = 0.0;
  twist.twist.twist.linear.z = 0.0;

  imu_pub->publish(imu);
  twist_pub->publish(twist);
  executor.spin_some();

  for (int i = 1; i <= 401; i++) {
    RCLCPP_INFO(node->get_logger(), "Count i: %d", i);
    twist.header.stamp = rclcpp::Clock().now();
    twist_pub->publish(twist);
    imu.header.stamp = rclcpp::Clock().now();
    imu_pub->publish(imu);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    node->test_on_timer();
    RCLCPP_INFO(
      node->get_logger(), "is_buffer_full: %d",
      node->get_estimation_module()->get_is_buffer_full());
    if (i < 400) {
      ASSERT_EQ(node->get_estimation_module()->get_buffer_size(), i);
      ASSERT_EQ(node->get_estimation_module()->get_is_buffer_full(), false);
    } else {
      ASSERT_EQ(node->get_estimation_module()->get_buffer_size(), 400);
      ASSERT_EQ(node->get_estimation_module()->get_is_buffer_full(), true);
    }
  }
}

TEST_F(GyroBiasEstimatorTestFixture, DT_1_3_2)
{
  sensor_msgs::msg::Imu imu;
  geometry_msgs::msg::TwistWithCovarianceStamped twist;
  auto node = std::make_shared<GyroBiasEstimatorTest>(0.000104, true);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  // 10秒後にexecutorを停止
  std::this_thread::sleep_for(std::chrono::seconds(10));
  executor.cancel();

  spin_thread.join();

  ASSERT_GE(node->get_estimation_module()->get_min_duration_ms(), 900);  // 最小経過時間は900ms以上
  ASSERT_LE(
    node->get_estimation_module()->get_max_duration_ms(), 1100);  // 最大経過時間は1100ms以下
}

// DT_1_4
TEST_F(GyroBiasEstimatorTestFixture, DT_1_4_1)
{
  sensor_msgs::msg::Imu imu;
  geometry_msgs::msg::TwistWithCovarianceStamped twist;
  auto node = std::make_shared<GyroBiasEstimatorTest>(0.000104, false);
  auto test_node = rclcpp::Node::make_shared("test_node");

  // Publish static TF
  publishStaticTransform(test_node);

  auto imu_pub = test_node->create_publisher<sensor_msgs::msg::Imu>(
    "/gyro_bias_validator/input/imu_raw", rclcpp::SensorDataQoS());
  auto twist_pub = test_node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/gyro_bias_validator/input/twist", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);

  imu.header.stamp = rclcpp::Clock().now();
  imu.header.frame_id = "imu_link";
  imu.angular_velocity.x = 0.0;
  imu.angular_velocity.y = 0.0;
  imu.angular_velocity.z = 0.0;

  twist.header.stamp = rclcpp::Clock().now();
  twist.header.frame_id = "base_link";
  twist.twist.twist.linear.x = 0.0;
  twist.twist.twist.linear.y = 0.0;
  twist.twist.twist.linear.z = 0.0;

  for (int i = 1; i < 41; i++) {
    RCLCPP_INFO(node->get_logger(), "Count i: %d", i);
    imu.header.stamp = rclcpp::Clock().now();
    twist.header.stamp = rclcpp::Clock().now();

    // iの値に応じてtwistの値を変更
    if (i <= 10) {
      twist.twist.twist.linear.x = 0.0;
    } else if (i <= 20) {
      twist.twist.twist.linear.x = 1.0;
    } else if (i <= 30) {
      twist.twist.twist.linear.x = 0.0;
    } else {
      twist.twist.twist.linear.x = -1.0;
    }

    twist_pub->publish(twist);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    imu_pub->publish(imu);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if (i == 10) {
      ASSERT_EQ(node->get_estimation_module()->get_buffer_size(), 10);
    }
    if (i == 11) {
      ASSERT_EQ(node->get_estimation_module()->get_buffer_size(), 0);
    }
    if (i == 20) {
      ASSERT_EQ(node->get_estimation_module()->get_buffer_size(), 0);
    }
    if (i == 30) {
      ASSERT_EQ(node->get_estimation_module()->get_buffer_size(), 10);
    }
    if (i == 31) {
      ASSERT_EQ(node->get_estimation_module()->get_buffer_size(), 0);
    }
    if (i == 40) {
      ASSERT_EQ(node->get_estimation_module()->get_buffer_size(), 0);
    }
  }
}

// DT_1_5
TEST_F(GyroBiasEstimatorTestFixture, DT_1_5_1)  // gyro_buffer_のx, y, zの標準偏差が閾値未満
{
  auto node = std::make_shared<GyroBiasEstimatorTest>(0.000104, false);  // 標準偏差の閾値を指定
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  boost::circular_buffer<geometry_msgs::msg::Vector3> gyro_buffer(400);

  // 異なるシード値で各軸のデータを生成
  const auto data_x = GenerateTestData(42, 0.0, 0.0001);  // 標準偏差 0.0001031615
  const auto data_y = GenerateTestData(43, 0.0, 0.0001);  // 標準偏差 0.0001007498
  const auto data_z = GenerateTestData(44, 0.0, 0.0001);  // 標準偏差 0.0000978264

  for (size_t i = 0; i < 400; ++i) {
    geometry_msgs::msg::Vector3 angular_velocity;
    angular_velocity.x = data_x[i];
    angular_velocity.y = data_y[i];
    angular_velocity.z = data_z[i];
    gyro_buffer.push_back(angular_velocity);
  }
  node->get_estimation_module()->set_gyro_buffer(gyro_buffer);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  executor.spin_some();

  ASSERT_EQ(node->get_estimation_module()->get_can_calibrate(), true);
}

TEST_F(GyroBiasEstimatorTestFixture, DT_1_5_2)  // gyro_buffer_のx, y, zの標準偏差が全てちょうど閾値
{
  auto node = std::make_shared<GyroBiasEstimatorTest>(0.0001031615, false);  // 標準偏差の閾値を指定
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  boost::circular_buffer<geometry_msgs::msg::Vector3> gyro_buffer(400);

  // 異なるシード値で各軸のデータを生成
  const auto data_x = GenerateTestData(42, 0.0, 0.0001);  // 標準偏差 0.0001031615
  const auto data_y = GenerateTestData(42, 0.0, 0.0001);  // 標準偏差 0.0001031615
  const auto data_z = GenerateTestData(42, 0.0, 0.0001);  // 標準偏差 0.0001031615

  for (size_t i = 0; i < 400; ++i) {
    geometry_msgs::msg::Vector3 angular_velocity;
    angular_velocity.x = data_x[i];
    angular_velocity.y = data_y[i];
    angular_velocity.z = data_z[i];
    gyro_buffer.push_back(angular_velocity);
  }
  node->get_estimation_module()->set_gyro_buffer(gyro_buffer);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  executor.spin_some();

  ASSERT_EQ(node->get_estimation_module()->get_can_calibrate(), true);
}

TEST_F(GyroBiasEstimatorTestFixture, DT_1_5_3)  // gyro_buffer_のx, y, zの標準偏差が閾値超
{
  auto node = std::make_shared<GyroBiasEstimatorTest>(0.000097, false);  // 標準偏差の閾値を指定
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  boost::circular_buffer<geometry_msgs::msg::Vector3> gyro_buffer(400);

  // 異なるシード値で各軸のデータを生成
  const auto data_x = GenerateTestData(42, 0.0, 0.0001);  // 標準偏差 0.0001031615
  const auto data_y = GenerateTestData(43, 0.0, 0.0001);  // 標準偏差 0.0001007498
  const auto data_z = GenerateTestData(44, 0.0, 0.0001);  // 標準偏差 0.0000978264

  for (size_t i = 0; i < 400; ++i) {
    geometry_msgs::msg::Vector3 angular_velocity;
    angular_velocity.x = data_x[i];
    angular_velocity.y = data_y[i];
    angular_velocity.z = data_z[i];
    gyro_buffer.push_back(angular_velocity);
  }
  node->get_estimation_module()->set_gyro_buffer(gyro_buffer);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  executor.spin_some();

  ASSERT_EQ(node->get_estimation_module()->get_can_calibrate(), false);
}

TEST_F(
  GyroBiasEstimatorTestFixture,
  DT_1_5_4)  // gyro_buffer_のxの標準偏差がちょうど閾値、y, zは閾値未満
{
  auto node = std::make_shared<GyroBiasEstimatorTest>(0.0001031615, false);  // 標準偏差の閾値を指定
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  boost::circular_buffer<geometry_msgs::msg::Vector3> gyro_buffer(400);

  // 異なるシード値で各軸のデータを生成
  const auto data_x = GenerateTestData(42, 0.0, 0.0001);  // 標準偏差 0.0001031615
  const auto data_y = GenerateTestData(43, 0.0, 0.0001);  // 標準偏差 0.0001007498
  const auto data_z = GenerateTestData(44, 0.0, 0.0001);  // 標準偏差 0.0000978264

  for (size_t i = 0; i < 400; ++i) {
    geometry_msgs::msg::Vector3 angular_velocity;
    angular_velocity.x = data_x[i];
    angular_velocity.y = data_y[i];
    angular_velocity.z = data_z[i];
    gyro_buffer.push_back(angular_velocity);
  }
  node->get_estimation_module()->set_gyro_buffer(gyro_buffer);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  executor.spin_some();

  ASSERT_EQ(node->get_estimation_module()->get_can_calibrate(), true);
}

TEST_F(GyroBiasEstimatorTestFixture, DT_1_5_5)  // gyro_buffer_のxの標準偏差が閾値超、y, zは閾値未満
{
  auto node = std::make_shared<GyroBiasEstimatorTest>(0.0001031614, false);  // 標準偏差の閾値を指定
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  boost::circular_buffer<geometry_msgs::msg::Vector3> gyro_buffer(400);

  // 異なるシード値で各軸のデータを生成
  const auto data_x = GenerateTestData(42, 0.0, 0.0001);  // 標準偏差 0.0001031615
  const auto data_y = GenerateTestData(43, 0.0, 0.0001);  // 標準偏差 0.0001007498
  const auto data_z = GenerateTestData(44, 0.0, 0.0001);  // 標準偏差 0.0000978264

  for (size_t i = 0; i < 400; ++i) {
    geometry_msgs::msg::Vector3 angular_velocity;
    angular_velocity.x = data_x[i];
    angular_velocity.y = data_y[i];
    angular_velocity.z = data_z[i];
    gyro_buffer.push_back(angular_velocity);
  }
  node->get_estimation_module()->set_gyro_buffer(gyro_buffer);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  executor.spin_some();

  ASSERT_EQ(node->get_estimation_module()->get_can_calibrate(), false);
}

TEST_F(
  GyroBiasEstimatorTestFixture,
  DT_1_5_6)  // gyro_buffer_のyの標準偏差がちょうど閾値、x, zは閾値未満
{
  auto node = std::make_shared<GyroBiasEstimatorTest>(0.0001031615, false);  // 標準偏差の閾値を指定
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  boost::circular_buffer<geometry_msgs::msg::Vector3> gyro_buffer(400);

  // 異なるシード値で各軸のデータを生成
  const auto data_x = GenerateTestData(43, 0.0, 0.0001);  // 標準偏差 0.0001007498
  const auto data_y = GenerateTestData(42, 0.0, 0.0001);  // 標準偏差 0.0001031615
  const auto data_z = GenerateTestData(44, 0.0, 0.0001);  // 標準偏差 0.0000978264

  for (size_t i = 0; i < 400; ++i) {
    geometry_msgs::msg::Vector3 angular_velocity;
    angular_velocity.x = data_x[i];
    angular_velocity.y = data_y[i];
    angular_velocity.z = data_z[i];
    gyro_buffer.push_back(angular_velocity);
  }
  node->get_estimation_module()->set_gyro_buffer(gyro_buffer);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  executor.spin_some();

  ASSERT_EQ(node->get_estimation_module()->get_can_calibrate(), true);
}

TEST_F(GyroBiasEstimatorTestFixture, DT_1_5_7)  // gyro_buffer_のyの標準偏差が閾値超、x, zは閾値未満
{
  auto node = std::make_shared<GyroBiasEstimatorTest>(0.0001031614, false);  // 標準偏差の閾値を指定
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  boost::circular_buffer<geometry_msgs::msg::Vector3> gyro_buffer(400);

  // 異なるシード値で各軸のデータを生成
  const auto data_x = GenerateTestData(43, 0.0, 0.0001);  // 標準偏差 0.0001007498
  const auto data_y = GenerateTestData(42, 0.0, 0.0001);  // 標準偏差 0.0001031615
  const auto data_z = GenerateTestData(44, 0.0, 0.0001);  // 標準偏差 0.0000978264

  for (size_t i = 0; i < 400; ++i) {
    geometry_msgs::msg::Vector3 angular_velocity;
    angular_velocity.x = data_x[i];
    angular_velocity.y = data_y[i];
    angular_velocity.z = data_z[i];
    gyro_buffer.push_back(angular_velocity);
  }
  node->get_estimation_module()->set_gyro_buffer(gyro_buffer);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  executor.spin_some();

  ASSERT_EQ(node->get_estimation_module()->get_can_calibrate(), false);
}

TEST_F(
  GyroBiasEstimatorTestFixture,
  DT_1_5_8)  // gyro_buffer_のzの標準偏差がちょうど閾値、x, yは閾値未満
{
  auto node = std::make_shared<GyroBiasEstimatorTest>(0.0001031615, false);  // 標準偏差の閾値を指定
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  boost::circular_buffer<geometry_msgs::msg::Vector3> gyro_buffer(400);

  // 異なるシード値で各軸のデータを生成
  const auto data_x = GenerateTestData(43, 0.0, 0.0001);  // 標準偏差 0.0001007498
  const auto data_y = GenerateTestData(44, 0.0, 0.0001);  // 標準偏差 0.0000978264
  const auto data_z = GenerateTestData(42, 0.0, 0.0001);  // 標準偏差 0.0001031615

  for (size_t i = 0; i < 400; ++i) {
    geometry_msgs::msg::Vector3 angular_velocity;
    angular_velocity.x = data_x[i];
    angular_velocity.y = data_y[i];
    angular_velocity.z = data_z[i];
    gyro_buffer.push_back(angular_velocity);
  }
  node->get_estimation_module()->set_gyro_buffer(gyro_buffer);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  executor.spin_some();

  ASSERT_EQ(node->get_estimation_module()->get_can_calibrate(), true);
}

TEST_F(GyroBiasEstimatorTestFixture, DT_1_5_9)  // gyro_buffer_のzの標準偏差が閾値超、x, yは閾値未満
{
  auto node = std::make_shared<GyroBiasEstimatorTest>(0.0001031614, false);  // 標準偏差の閾値を指定
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  boost::circular_buffer<geometry_msgs::msg::Vector3> gyro_buffer(400);

  // 異なるシード値で各軸のデータを生成
  const auto data_x = GenerateTestData(43, 0.0, 0.0001);  // 標準偏差 0.0001007498
  const auto data_y = GenerateTestData(44, 0.0, 0.0001);  // 標準偏差 0.0000978264
  const auto data_z = GenerateTestData(42, 0.0, 0.0001);  // 標準偏差 0.0001031615

  for (size_t i = 0; i < 400; ++i) {
    geometry_msgs::msg::Vector3 angular_velocity;
    angular_velocity.x = data_x[i];
    angular_velocity.y = data_y[i];
    angular_velocity.z = data_z[i];
    gyro_buffer.push_back(angular_velocity);
  }
  node->get_estimation_module()->set_gyro_buffer(gyro_buffer);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  executor.spin_some();

  ASSERT_EQ(node->get_estimation_module()->get_can_calibrate(), false);
}

// DT_1_6
TEST_F(GyroBiasEstimatorTestFixture, DT_1_6_1)
{
  int count = 0;
  double gyro_bias_x = 0.0;
  double gyro_bias_y = 0.0;
  double gyro_bias_z = 0.0;
  auto node = std::make_shared<GyroBiasEstimatorTest>(0.0001031615, false);
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto subscriber = test_node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/gyro_bias_validator/output/gyro_bias", rclcpp::SensorDataQoS(),
    [&count, &gyro_bias_x, &gyro_bias_y,
     &gyro_bias_z](const geometry_msgs::msg::Vector3Stamped msg) {
      RCLCPP_INFO(rclcpp::get_logger("test"), "Received message");
      count++;
      gyro_bias_x = msg.vector.x;
      gyro_bias_y = msg.vector.y;
      gyro_bias_z = msg.vector.z;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);

  boost::circular_buffer<geometry_msgs::msg::Vector3> gyro_buffer(400);

  // 異なるシード値で各軸のデータを生成
  gyro_buffer.clear();
  const auto data_x = GenerateTestData(43, 1.0, 0.0001);  // 標準偏差 0.0001007498
  const auto data_y = GenerateTestData(44, 2.0, 0.0001);  // 標準偏差 0.0000978264
  const auto data_z = GenerateTestData(42, 3.0, 0.0001);  // 標準偏差 0.0001031615
  double median_x = CalculateMedian(data_x);
  double median_y = CalculateMedian(data_y);
  double median_z = CalculateMedian(data_z);
  geometry_msgs::msg::Vector3 angular_velocity;
  for (size_t i = 0; i < 399; ++i) {
    angular_velocity.x = data_x[i];
    angular_velocity.y = data_y[i];
    angular_velocity.z = data_z[i];
    gyro_buffer.push_back(angular_velocity);
  }

  node->get_estimation_module()->set_gyro_buffer(gyro_buffer);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.spin_some();
  ASSERT_EQ(count, 0);

  angular_velocity.x = data_x[399];
  angular_velocity.y = data_y[399];
  angular_velocity.z = data_z[399];
  gyro_buffer.push_back(angular_velocity);
  node->get_estimation_module()->set_gyro_buffer(gyro_buffer);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.spin_some();
  ASSERT_EQ(count, 1);
  ASSERT_EQ(gyro_bias_x, median_x);
  ASSERT_EQ(gyro_bias_y, median_y);
  ASSERT_EQ(gyro_bias_z, median_z);
}
