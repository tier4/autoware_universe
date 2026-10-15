// Copyright 2026 TIER IV, Inc.
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

#include "autoware/ml_planner/utils/model_metadata.hpp"

#include <openssl/evp.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

namespace autoware::ml_planner::utils
{

namespace
{
constexpr const char * SHA256_KEY = "onnx_sha256";
constexpr const char * MAJOR_VERSION_KEY = "major_version";

std::string to_lower(std::string str)
{
  std::transform(str.begin(), str.end(), str.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return str;
}
}  // namespace

std::string model_metadata_path(const std::string & model_path)
{
  return (std::filesystem::path(model_path).parent_path() / MODEL_METADATA_FILENAME).string();
}

nlohmann::json load_model_metadata(const std::string & metadata_path)
{
  std::ifstream ifs(metadata_path);
  if (!ifs) {
    throw std::runtime_error("Failed to open model metadata: " + metadata_path);
  }
  try {
    return nlohmann::json::parse(ifs);
  } catch (const nlohmann::json::parse_error & e) {
    throw std::runtime_error(
      "Failed to parse model metadata " + metadata_path + ": " + std::string(e.what()));
  }
}

std::string compute_file_sha256_hex(const std::string & path)
{
  constexpr std::size_t READ_BUFFER_BYTES = 64 * 1024;
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    throw std::runtime_error("Failed to open file for SHA-256: " + path);
  }

  const std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(
    EVP_MD_CTX_new(), &EVP_MD_CTX_free);
  if (!ctx || EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("Failed to initialize SHA-256");
  }

  std::array<char, READ_BUFFER_BYTES> buffer{};
  while (ifs) {
    ifs.read(buffer.data(), buffer.size());
    const std::streamsize n = ifs.gcount();
    if (n > 0 && EVP_DigestUpdate(ctx.get(), buffer.data(), static_cast<std::size_t>(n)) != 1) {
      throw std::runtime_error("Failed to update SHA-256: " + path);
    }
  }
  if (ifs.bad()) {
    throw std::runtime_error("Failed to read file for SHA-256: " + path);
  }

  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(ctx.get(), digest.data(), &digest_size) != 1) {
    throw std::runtime_error("Failed to finalize SHA-256: " + path);
  }

  constexpr std::array<char, 16> HEX = {'0', '1', '2', '3', '4', '5', '6', '7',
                                        '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
  std::string hex;
  hex.reserve(digest_size * 2);
  for (unsigned int i = 0; i < digest_size; ++i) {
    hex.push_back(HEX[digest[i] >> 4]);
    hex.push_back(HEX[digest[i] & 0x0f]);
  }
  return hex;
}

std::string verify_model_sha256(const nlohmann::json & metadata, const std::string & model_path)
{
  const auto it = metadata.find(SHA256_KEY);
  if (it == metadata.end() || !it->is_string()) {
    throw std::runtime_error(
      std::string("Model metadata must contain a string field \"") + SHA256_KEY + "\"");
  }
  const std::string expected = to_lower(it->get<std::string>());
  const std::string actual = compute_file_sha256_hex(model_path);
  if (expected != actual) {
    throw std::runtime_error(
      "SHA-256 mismatch for " + model_path + ": metadata=" + expected + ", actual=" + actual);
  }
  return actual;
}

void verify_model_major_version(const nlohmann::json & metadata, const int expected_major_version)
{
  const auto it = metadata.find(MAJOR_VERSION_KEY);
  if (it == metadata.end() || !it->is_number_integer()) {
    throw std::runtime_error(
      std::string("Model metadata must contain an integer field \"") + MAJOR_VERSION_KEY + "\"");
  }
  const int major_version = it->get<int>();
  if (major_version != expected_major_version) {
    throw std::runtime_error(
      "Incompatible model major version " + std::to_string(major_version) +
      ". This node requires major version " + std::to_string(expected_major_version) + ".");
  }
}

}  // namespace autoware::ml_planner::utils
