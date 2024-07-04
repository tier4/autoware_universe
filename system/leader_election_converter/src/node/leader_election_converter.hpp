// Copyright 2024 The Autoware Contributors
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

#ifndef NODE__LEADER_ELECTION_CONVERTER_HPP_
#define NODE__LEADER_ELECTION_CONVERTER_HPP_

#include "availability_converter.hpp"
#include "log_converter.hpp"
#include "mrm_converter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace leader_election_converter
{

class LeaderElectionConverter : public rclcpp::Node
{
public:
  explicit LeaderElectionConverter(const rclcpp::NodeOptions & node_options);

private:
  std::string availability_dest_ip_;
  std::string availability_dest_port_;
  std::string mrm_state_dest_ip_;
  std::string mrm_state_dest_port_;
  std::string mrm_request_src_ip_;
  std::string mrm_request_src_port_;
  std::string election_communication_src_ip_;
  std::string election_communication_src_port_;
  std::string election_status_src_ip_;
  std::string election_status_src_port_;

  AvailabilityConverter availability_converter_;
  MrmConverter mrm_converter_;
  LogConverter log_converter_;
};

}  // namespace leader_election_converter

#endif  // NODE__LEADER_ELECTION_CONVERTER_HPP_
