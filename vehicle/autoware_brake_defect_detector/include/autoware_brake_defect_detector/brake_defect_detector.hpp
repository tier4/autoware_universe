// Copyright 2026 The Autoware Contributors
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

#ifndef AUTOWARE_BRAKE_DEFECT_DETECTOR__BRAKE_DEFECT_DETECTOR_HPP_
#define AUTOWARE_BRAKE_DEFECT_DETECTOR__BRAKE_DEFECT_DETECTOR_HPP_

#include <deque>
#include <optional>

namespace autoware::brake_defect_detector
{

struct DetectorParams
{
  double actuation_delay_sec{0.20};
  double cusum_drift_k{0.15};
  double cusum_threshold_h{0.05};
  double min_speed_mps{1.5};
  double max_decel_cmd{-0.1};
  double brake_cmd_min{0.05};
  double brake_cmd_max{0.35};
  double jerk_limit_mps3{3.0};
  double settling_time_sec{0.5};
  double residual_filter_tau_sec{0.25};
  double max_update_gap_sec{0.5};
};

struct DiagnosticStatus
{
  bool brake_defect_detected{false};
  double filtered_residual{0.0};
  double cusum_statistic{0.0};
  bool valid_condition{false};
  bool data_ready{false};
};

class BrakeDefectDetector
{
public:
  explicit BrakeDefectDetector(const DetectorParams & params);

  void reset();
  void observe_command(double acceleration, double time_sec);
  DiagnosticStatus update(
    double acceleration_command, double brake_command, double measured_acceleration, double speed,
    double pitch_rad, double time_sec);

private:
  struct TimedCommand
  {
    double time_sec;
    double acceleration;
  };

  DetectorParams params_;
  std::deque<TimedCommand> commands_;
  std::optional<double> last_update_time_;
  double settling_until_sec_{0.0};
  double filtered_residual_{0.0};
  double cusum_statistic_{0.0};
  bool filter_initialized_{false};
};

}  // namespace autoware::brake_defect_detector

#endif  // AUTOWARE_BRAKE_DEFECT_DETECTOR__BRAKE_DEFECT_DETECTOR_HPP_
