// Copyright 2026 TIER IV, Inc.

#include "autoware/static_freespace_planner/waypoint_loader.hpp"

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
    waypoints.push_back(parseCSVLine(line));
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
    Waypoint wp = parseCSVLine(line);
    if (std::find(available_seqs.begin(), available_seqs.end(), wp.seq) == available_seqs.end()) {
      available_seqs.push_back(wp.seq);
    }
  }

  file.close();
  // Sort and remove duplicates
  std::sort(available_seqs.begin(), available_seqs.end());
  available_seqs.erase(
    std::unique(available_seqs.begin(), available_seqs.end()), available_seqs.end());
  return available_seqs;
}

WaypointLoader::Waypoint WaypointLoader::loadFirstWaypoint(const std::string & csv_path)
{
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + csv_path);
  }

  std::string line;
  // Skip header line
  std::getline(file, line);

  if (std::getline(file, line)) {
    file.close();
    return parseCSVLine(line);
  } else {
    file.close();
    throw std::runtime_error("No waypoints found in file: " + csv_path);
  }
}

WaypointLoader::Waypoint WaypointLoader::loadLastWaypoint(const std::string & csv_path)
{
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + csv_path);
  }

  std::string line;
  // Skip header line
  std::getline(file, line);

  Waypoint last_waypoint;
  bool found = false;
  while (std::getline(file, line)) {
    last_waypoint = parseCSVLine(line);
    found = true;
  }

  file.close();
  if (found) {
    return last_waypoint;
  } else {
    throw std::runtime_error("No waypoints found in file: " + csv_path);
  }
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
    Waypoint wp = parseCSVLine(line);
    if (wp.seq == seq) {
      waypoints.push_back(wp);
    }
  }

  file.close();
  return waypoints;
}

WaypointLoader::Waypoint WaypointLoader::parseCSVLine(const std::string & line)
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
    wp = Waypoint{};
    return wp;
  }

  // Parse each item
  wp.seq = std::stoi(items[0]);
  wp.x = std::stod(items[1]);
  wp.y = std::stod(items[2]);
  wp.z = std::stod(items[3]);
  wp.qx = std::stod(items[4]);
  wp.qy = std::stod(items[5]);
  wp.qz = std::stod(items[6]);
  wp.qw = std::stod(items[7]);
  wp.mps = std::stod(items[8]);

  return wp;
}
}  // namespace autoware::static_freespace_planner
