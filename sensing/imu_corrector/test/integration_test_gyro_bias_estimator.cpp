#include "integration_test_gyro_bias_estimator.hpp"

using namespace std::chrono_literals;

void TestGyroBiasEstimator::SetUp()
{
  rclcpp::init(0, nullptr);

  test_node_ = std::make_shared<rclcpp::Node>("test_node");

  imu_pub_ =
    test_node_->create_publisher<sensor_msgs::msg::Imu>("/imu_raw", rclcpp::SensorDataQoS());
  twist_pub_ = test_node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/twist", rclcpp::SensorDataQoS());

  received_bias_msgs_.clear();
  bias_sub_ = test_node_->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/gyro_bias", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
      received_bias_msgs_.push_back(*msg);
    });

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(test_node_);
}

void TestGyroBiasEstimator::TearDown()
{
  executor_.reset();
  test_node_.reset();
  rclcpp::shutdown();
}

TEST_F(TestGyroBiasEstimator, TestGyroBiasEstimation)
{
  const double test_duration = 30.0;
  const double dt = 0.1;
  const int num_messages = static_cast<int>(test_duration / dt);

  for (int i = 0; i < num_messages; ++i) {
    const auto current_time = test_node_->now();

    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = current_time;
    imu_msg->angular_velocity.x = 0.1;
    imu_msg->angular_velocity.y = 0.2;
    imu_msg->angular_velocity.z = 0.3;
    imu_pub_->publish(*imu_msg);

    auto twist_msg = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
    twist_msg->header.stamp = current_time;
    twist_msg->twist.twist.linear.x = 0.0;
    twist_pub_->publish(*twist_msg);

    executor_->spin_some();
    rclcpp::sleep_for(100ms);
  }

  ASSERT_GT(received_bias_msgs_.size(), 0) << "No bias messages received";

  const auto & last_bias = received_bias_msgs_.back().vector;
  EXPECT_NEAR(last_bias.x, 0.1, 0.01);
  EXPECT_NEAR(last_bias.y, 0.2, 0.01);
  EXPECT_NEAR(last_bias.z, 0.3, 0.01);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
