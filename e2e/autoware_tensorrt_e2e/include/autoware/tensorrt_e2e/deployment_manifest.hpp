// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#pragma once
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <sstream>
#include <stdexcept>
namespace autoware::tensorrt_e2e {
inline std::string file_sha256(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input)
    throw std::runtime_error("Cannot read deployment artifact: " +
                             path.string());
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(),
                                                              EVP_MD_CTX_free);
  if (!ctx || EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("Cannot initialize SHA256");
  std::array<char, 65536> buffer;
  while (input.read(buffer.data(), buffer.size()) || input.gcount()) {
    if (EVP_DigestUpdate(ctx.get(), buffer.data(), input.gcount()) != 1)
      throw std::runtime_error("Cannot hash artifact");
  }
  if (!input.eof())
    throw std::runtime_error("Artifact read failed");
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest;
  unsigned int length = 0;
  if (EVP_DigestFinal_ex(ctx.get(), digest.data(), &length) != 1)
    throw std::runtime_error("Cannot finalize SHA256");
  std::ostringstream output;
  for (unsigned int i = 0; i < length; ++i)
    output << std::hex << std::setfill('0') << std::setw(2)
           << static_cast<int>(digest[i]);
  return output.str();
}
inline bool engine_identity_matches(const std::string &onnx,
                                    const std::string &engine,
                                    const std::string &precision = "default") {
  std::ifstream input(engine + ".identity");
  std::string graph_hash, engine_hash, saved_precision;
  if (!(input >> graph_hash >> engine_hash >> saved_precision))
    return false;
  return saved_precision == precision && graph_hash == file_sha256(onnx) &&
         engine_hash == file_sha256(engine);
}

inline void record_engine_identity(const std::string &onnx,
                                   const std::string &engine,
                                   const std::string &precision = "default") {
  const std::string temporary = engine + ".identity.tmp";
  std::ofstream output(temporary);
  output << file_sha256(onnx) << "\n"
         << file_sha256(engine) << "\n"
         << precision << "\n";
  output.close();
  if (!output)
    throw std::runtime_error("Cannot write engine identity: " + temporary);
  std::filesystem::rename(temporary, engine + ".identity");
}

inline nlohmann::json
validate_deployment_manifest(const std::filesystem::path &path) {
  std::ifstream input(path);
  if (!input)
    throw std::runtime_error("Missing deployment manifest: " + path.string());
  const auto manifest = nlohmann::json::parse(input);
  if (manifest.at("schema_version") != 1 || !manifest.at("files").is_object() ||
      !manifest.at("parameters").is_object())
    throw std::runtime_error("Invalid deployment manifest");
  for (const auto &[name, digest] : manifest.at("files").items()) {
    const std::filesystem::path file(name);
    if (file.filename() != file || name.empty() || name == "." || name == "..")
      throw std::runtime_error(
          "Manifest files must be package-local basenames");
    if (file_sha256(path.parent_path() / file) != digest.get<std::string>())
      throw std::runtime_error("Deployment artifact SHA256 mismatch: " + name);
  }
  for (const char *key : {"planner_file", "extractor_file"})
    if (!manifest.at("files").contains(manifest.at(key).get<std::string>()))
      throw std::runtime_error("Manifest does not hash the selected graph");
  return manifest;
}
} // namespace autoware::tensorrt_e2e
