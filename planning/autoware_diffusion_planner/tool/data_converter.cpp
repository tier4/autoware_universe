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

#include "rosbag_parser.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iostream>

int main(int argc, char ** argv)
{
  std::cout << "data_converter" << std::endl;

  if (argc != 2) {
    std::cerr << "Usage: ndt_ekf_simulator <input_bag_path>" << std::endl;
    return 1;
  }
  const std::string input_bag_path = argv[1];

  rosbag_parser::RosbagParser rosbag_parser(input_bag_path);
  rosbag_parser.create_reader(input_bag_path);

  while (rosbag_parser.has_next()) {
    const rosbag2_storage::SerializedBagMessageSharedPtr msg = rosbag_parser.read_next();
    std::cout << msg->time_stamp << " " << msg->topic_name << " " << std::endl;
  }
}
