// Copyright 2026 TIER IV, Inc.

#include "autoware/static_freespace_planner/waypoint_loader.hpp"
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::static_freespace_planner
{
WaypointLoader::WaypointLoader() = default;
std::vector<WaypointLoader::Waypoint> WaypointLoader::loadWaypoints(const std::string & csv_path)
{
  std::vector<Waypoint> waypoints;
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    return std::vector<Waypoint>();
  }

  std::string line;
  // Skip header line
  std::getline(file, line);

  while (std::getline(file, line)) {
    auto wp_opt = parseCSVLine(line);
    if (wp_opt) {
      waypoints.push_back(*wp_opt);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("WaypointLoader"),
        "Skipping invalid waypoint at line %zu in file '%s'", waypoints.size() + 2, csv_path.c_str());
      return std::vector<Waypoint>();  // Return empty if any line is invalid
    }
  }

  file.close();
  return waypoints;
}

std::vector<int> WaypointLoader::getAvailableSeqs(const std::string & csv_path)
{
  std::vector<int> available_seqs;
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    return available_seqs;
  }

  std::string line;
  // Skip header line
  std::getline(file, line);

  while (std::getline(file, line)) {
    auto wp_opt = parseCSVLine(line);
    if (wp_opt) {
      const auto & wp = *wp_opt;
      if (std::find(available_seqs.begin(), available_seqs.end(), wp.seq) == available_seqs.end()) {
        available_seqs.push_back(wp.seq);
      }
    } else {
      RCLCPP_WARN(rclcpp::get_logger("WaypointLoader"),
        "Skipping invalid waypoint at line %zu in file '%s' while loading available seqs", available_seqs.size() + 2, csv_path.c_str());
      return available_seqs;  // Return empty if any line is invalid
    }
  }

  file.close();
  // Sort and remove duplicates
  std::sort(available_seqs.begin(), available_seqs.end());
  available_seqs.erase(
    std::unique(available_seqs.begin(), available_seqs.end()), available_seqs.end());
  return available_seqs;
}

std::vector<WaypointLoader::Waypoint> WaypointLoader::loadWaypointsBySeq(
  const std::string & csv_path, int seq)
{
  std::vector<Waypoint> waypoints;
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    return std::vector<Waypoint>();
  }

  std::string line;
  // Skip header line
  std::getline(file, line);

  while (std::getline(file, line)) {
    auto wp_opt = parseCSVLine(line);
    if (wp_opt && wp_opt->seq == seq) {
      waypoints.push_back(*wp_opt);
    }
  }

  file.close();
  return waypoints;
}

std::optional<WaypointLoader::Waypoint> WaypointLoader::parseCSVLine(const std::string & line)
{
  std::stringstream ss(line);
  std::string item;
  std::vector<std::string> items;
  Waypoint wp;

  // Split line by comma
  while (std::getline(ss, item, ',')) {
    items.push_back(item);
  }

  // Check if items size is valid
  if (items.size() < 9) {
    return std::nullopt;
  }

  try {
    // seq
    wp.seq = std::stoi(items[0]);

    // x, y, z
    wp.x  = std::stod(items[1]);
    wp.y  = std::stod(items[2]);
    wp.z  = std::stod(items[3]);

    // qx, qy, qz, qw
    wp.qx = std::stod(items[4]);
    wp.qy = std::stod(items[5]);
    wp.qz = std::stod(items[6]);
    wp.qw = std::stod(items[7]);

    // mps
    wp.mps = std::stod(items[8]);
  }
  catch (...) {
    return std::nullopt;
  }

  return wp;
}
}  // namespace autoware::static_freespace_planner
