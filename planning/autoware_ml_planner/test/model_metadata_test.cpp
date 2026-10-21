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

#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace autoware::ml_planner::test
{

namespace
{
// SHA-256 of "abc" (FIPS 180-2 test vector).
constexpr const char * ABC_SHA256 =
  "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";
}  // namespace

class ModelMetadataTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = std::filesystem::temp_directory_path() /
           ("ml_planner_model_metadata_test_" + std::to_string(::getpid()));
    std::filesystem::create_directories(dir_);
    model_path_ = (dir_ / "model.onnx").string();
    std::ofstream(model_path_, std::ios::binary) << "abc";
  }

  void TearDown() override { std::filesystem::remove_all(dir_); }

  void write_metadata(const std::string & content) const
  {
    std::ofstream(dir_ / "metadata.json") << content;
  }

  std::filesystem::path dir_;
  std::string model_path_;
};

TEST_F(ModelMetadataTest, MetadataPathIsNextToModel)
{
  EXPECT_EQ(utils::model_metadata_path(model_path_), (dir_ / "metadata.json").string());
}

TEST_F(ModelMetadataTest, ComputeSha256)
{
  EXPECT_EQ(utils::compute_file_sha256_hex(model_path_), ABC_SHA256);
}

TEST_F(ModelMetadataTest, LoadAndVerify)
{
  write_metadata(
    std::string(R"({"major_version": 5, "onnx_sha256": ")") + ABC_SHA256 + R"(", "note": "x"})");
  const auto metadata = utils::load_model_metadata(utils::model_metadata_path(model_path_));
  EXPECT_EQ(utils::verify_model_sha256(metadata, model_path_), ABC_SHA256);
  EXPECT_NO_THROW(utils::verify_model_major_version(metadata, 5));
  EXPECT_THROW(utils::verify_model_major_version(metadata, 6), std::runtime_error);
}

TEST_F(ModelMetadataTest, Sha256IsCaseInsensitive)
{
  std::string upper = ABC_SHA256;
  for (auto & c : upper) c = static_cast<char>(std::toupper(c));
  const auto metadata = nlohmann::json{{"onnx_sha256", upper}};
  EXPECT_NO_THROW(utils::verify_model_sha256(metadata, model_path_));
}

TEST_F(ModelMetadataTest, Sha256Mismatch)
{
  const auto metadata = nlohmann::json{{"onnx_sha256", std::string(64, '0')}};
  EXPECT_THROW(utils::verify_model_sha256(metadata, model_path_), std::runtime_error);
}

TEST_F(ModelMetadataTest, MissingFields)
{
  const auto metadata = nlohmann::json::object();
  EXPECT_THROW(utils::verify_model_sha256(metadata, model_path_), std::runtime_error);
  EXPECT_THROW(utils::verify_model_major_version(metadata, 5), std::runtime_error);
}

TEST_F(ModelMetadataTest, InvalidJson)
{
  write_metadata("{not json");
  EXPECT_THROW(
    utils::load_model_metadata(utils::model_metadata_path(model_path_)), std::runtime_error);
}

TEST_F(ModelMetadataTest, MissingMetadataFile)
{
  EXPECT_THROW(
    utils::load_model_metadata(utils::model_metadata_path(model_path_)), std::runtime_error);
}

}  // namespace autoware::ml_planner::test
