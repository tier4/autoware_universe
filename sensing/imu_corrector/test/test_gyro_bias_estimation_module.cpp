#include "test_fixture_gyro_bias_estimation_module.hpp"

namespace imu_corrector
{
<<<<<<< Updated upstream

class GyroBiasEstimationModuleTestWrapper : public GyroBiasEstimationModule
{
public:
  GyroBiasEstimationModuleTestWrapper(
    double velocity_threshold, double timestamp_threshold, size_t data_num_threshold,
    double bias_change_threshold, const rclcpp::Logger & logger,
    const std::shared_ptr<rclcpp::Clock> & clock)
  : GyroBiasEstimationModule(
      velocity_threshold, timestamp_threshold, data_num_threshold, bias_change_threshold, logger,
      clock)
  {
  }
  size_t get_buffer_size() const { return gyro_buffer_.size(); }
};

class GyroBiasEstimationModuleTest : public ::testing::Test
{
public:
  double velocity_threshold = 1.0;
  double timestamp_threshold = 0.1;
  size_t data_num_threshold = 5;
  double bias_change_threshold = 0.00061;
  rclcpp::Logger mock_logger_{rclcpp::get_logger("GyroBiasEstimationModuleTest")};
  std::shared_ptr<rclcpp::Clock> mock_clock_{std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)};

  GyroBiasEstimationModuleTestWrapper module = GyroBiasEstimationModuleTestWrapper(
    velocity_threshold, timestamp_threshold, data_num_threshold, bias_change_threshold,
    mock_logger_, mock_clock_);
};

=======
>>>>>>> Stashed changes
TEST_F(GyroBiasEstimationModuleTest, GetBiasEstimationWhenVehicleStopped)
{
  geometry_msgs::msg::Vector3 gyro;
  gyro.x = 0.1;
  gyro.y = 0.2;
  gyro.z = 0.3;
  for (size_t i = 0; i < data_num_threshold + 1; ++i) {
    module.update_velocity(
      i * 0.1 * timestamp_threshold, 0.0);  // velocity = 0.0 < 1.0 = velocity_threshold
    module.update_gyro(i * 0.1 * timestamp_threshold, gyro);
  }
  ASSERT_NEAR(module.get_bias().x, gyro.x, 0.0001);
  ASSERT_NEAR(module.get_bias().y, gyro.y, 0.0001);
  ASSERT_NEAR(module.get_bias().z, gyro.z, 0.0001);
}

TEST_F(GyroBiasEstimationModuleTest, GetInsufficientDataException)
{
  ASSERT_THROW(module.get_bias(), std::runtime_error);
}

TEST_F(GyroBiasEstimationModuleTest, GetInsufficientDataExceptionWhenVehicleMoving)
{
  geometry_msgs::msg::Vector3 gyro;
  gyro.x = 0.1;
  gyro.y = 0.2;
  gyro.z = 0.3;
  for (size_t i = 0; i < data_num_threshold + 1; ++i) {
    module.update_velocity(
      i * 0.1 * timestamp_threshold, 5.0);  // velocity = 5.0 > 1.0 = velocity_threshold
    module.update_gyro(i * 0.1 * timestamp_threshold, gyro);
  }
  ASSERT_THROW(module.get_bias(), std::runtime_error);
}

// FSR01-3
TEST_F(GyroBiasEstimationModuleTest, RecordsImuDataToBuffer)
{
  geometry_msgs::msg::Vector3 gyro;
  gyro.x = 0.1;
  gyro.y = 0.2;
  gyro.z = 0.3;
  module.update_velocity(0.0, 0.0);
  module.update_gyro(0.0, gyro);
  ASSERT_EQ(module.get_buffer_size(), 1);
}

<<<<<<< Updated upstream
=======
// 保留

>>>>>>> Stashed changes
// FSR01-4
TEST_F(GyroBiasEstimationModuleTest, ClearsBufferWhenVehicleMoving)
{
  // 初期状態で停止中のデータを追加
  geometry_msgs::msg::Vector3 gyro;
  gyro.x = 0.1;
  gyro.y = 0.2;
  gyro.z = 0.3;
  module.update_velocity(0.0, 0.0);  // 速度0.0で停止中
  module.update_gyro(0.0, gyro);
  ASSERT_EQ(module.get_buffer_size(), 1);  // バッファにデータが1つ追加されていることを確認

  // 車両が動き出した状態を作る
  module.update_velocity(0.1, 2.0);        // 速度2.0で移動（> velocity_threshold）
  module.update_gyro(0.1, gyro);           // 車両が動いている状態でデータを追加
  ASSERT_EQ(module.get_buffer_size(), 0);  // バッファがクリアされていることを確認

  module.update_velocity(0.0, 0.0);  // 速度0.0で停止中
  module.update_gyro(0.0, gyro);
  ASSERT_EQ(module.get_buffer_size(), 1);  // バッファにデータが1つ追加されていることを確認
}

// FSR01-5
TEST_F(GyroBiasEstimationModuleTest, CheckCalibrationPossibleWithin2Seconds)
{
  geometry_msgs::msg::Vector3 gyro;

  // 2秒間分のデータを0.1秒間隔で追加（停止状態）
  for (size_t i = 0; i < data_num_threshold; ++i) {
    gyro.x = 0.1 + i * 0.05;
    gyro.y = 0.2 - i * 0.03;
    gyro.z = 0.3 + ((i % 2 == 0) ? 0.1 : -0.1);
    module.update_velocity(i * 0.1, 0.0);  // 停止状態（velocity = 0.0）
    module.update_gyro(i * 0.1, gyro);
  }
  module.get_bias();
  // キャリブレーション可能な状態であることを確認
  ASSERT_FALSE(module.get_is_calibration_possible());

  gyro.x = 0.0;
  gyro.y = 0.0;
  gyro.z = 0.0;
  for (size_t i = 0; i < data_num_threshold; ++i) {
    module.update_velocity(i * 0.1, 0.0);  // 停止状態（velocity = 0.0）
    module.update_gyro(i * 0.1, gyro);
  }
  module.get_bias();
}

// FSR01-6

// FSR01-7
<<<<<<< Updated upstream
=======
// subscribeしたtopicがメンバ変数に格納されることを確認
>>>>>>> Stashed changes

}  // namespace imu_corrector
