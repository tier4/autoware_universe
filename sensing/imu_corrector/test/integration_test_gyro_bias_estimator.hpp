#ifndef INTEGRATION_TEST_GYRO_BIAS_ESTIMATOR_HPP_
#define INTEGRATION_TEST_GYRO_BIAS_ESTIMATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

class TestGyroBiasEstimator : public ::testing::Test
{
protected:
  void SetUp() override;
  void TearDown() override;

  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr bias_sub_;
  std::vector<geometry_msgs::msg::Vector3Stamped> received_bias_msgs_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

#endif  // INTEGRATION_TEST_GYRO_BIAS_ESTIMATOR_HPP_
