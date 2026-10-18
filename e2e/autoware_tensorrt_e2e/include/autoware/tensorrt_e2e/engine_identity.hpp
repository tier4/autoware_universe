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

#pragma once
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <memory>
#include <openssl/evp.h>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
namespace autoware::tensorrt_e2e {
inline std::string file_sha256(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input)
    throw std::runtime_error("Cannot read: " + path.string());
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(),
                                                              EVP_MD_CTX_free);
  if (!ctx || EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("Cannot initialize SHA256");
  std::array<char, 65536> buffer;
  while (input.read(buffer.data(), buffer.size()) || input.gcount()) {
    if (EVP_DigestUpdate(ctx.get(), buffer.data(), input.gcount()) != 1)
      throw std::runtime_error("Cannot hash file");
  }
  if (!input.eof())
    throw std::runtime_error("File read failed");
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

// Replace `target` with `content` whole. The temporary file is unique to this
// call (pid, thread, random suffix, created exclusively) and sits beside the
// target, so the rename is atomic and several processes may write the same
// target at once: each reader sees one complete file, and the last writer wins.
inline void write_file_atomically(const std::filesystem::path &target,
                                  const std::string &content) {
  std::random_device random;
  std::ostringstream name;
  name << target.string() << ".tmp." << ::getpid() << "."
       << std::hash<std::thread::id>{}(std::this_thread::get_id()) << ".";
  std::string temporary;
  int fd = -1;
  for (int attempt = 0; fd < 0 && attempt < 16; ++attempt) {
    std::ostringstream candidate;
    candidate << name.str() << std::hex << random();
    temporary = candidate.str();
    fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0666);
    if (fd < 0 && errno != EEXIST)
      break;
  }
  if (fd < 0)
    throw std::runtime_error("Cannot create " + temporary + ": " +
                             std::strerror(errno));
  ::close(fd);
  std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
  output << content;
  output.close();
  if (!output || std::rename(temporary.c_str(), target.c_str()) != 0) {
    const int error = errno;
    std::remove(temporary.c_str());
    throw std::runtime_error("Cannot write " + target.string() + ": " +
                             std::strerror(error));
  }
}

// What the identity file of `engine` holds, or nothing if the engine is gone.
inline std::optional<std::string>
engine_identity_text(const std::string &onnx, const std::string &engine,
                     const std::string &precision) {
  try {
    return file_sha256(onnx) + "\n" + file_sha256(engine) + "\n" + precision +
           "\n";
  } catch (const std::runtime_error &) {
    if (std::filesystem::exists(engine))
      throw;
    return std::nullopt;
  }
}

// Records which graph and engine bytes belong together. A load whose identity
// already matches writes nothing, so any number of processes can open one
// cached engine at once. After a build it writes the file atomically and reads
// it back: a process that built the same engine concurrently may have replaced
// the engine meanwhile, and whichever records last re-hashes the engine then on
// disk, so the pair ends consistent. An engine removed meanwhile (another
// process found it stale) is left to the process that rebuilds it.
inline void record_engine_identity(const std::string &onnx,
                                   const std::string &engine,
                                   const std::string &precision = "default") {
  const std::string path = engine + ".identity";
  for (int attempt = 0; attempt < 8; ++attempt) {
    const auto expected = engine_identity_text(onnx, engine, precision);
    if (!expected)
      return;
    std::ifstream input(path, std::ios::binary);
    const std::string saved((std::istreambuf_iterator<char>(input)),
                            std::istreambuf_iterator<char>());
    if (saved == *expected)
      return;
    write_file_atomically(path, *expected);
  }
}
} // namespace autoware::tensorrt_e2e
