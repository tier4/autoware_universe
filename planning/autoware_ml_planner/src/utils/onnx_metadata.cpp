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

#include "autoware/ml_planner/utils/onnx_metadata.hpp"

#include <cstdint>
#include <fstream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::ml_planner::utils
{
namespace
{
// Protobuf wire types
constexpr uint32_t WIRE_VARINT = 0;
constexpr uint32_t WIRE_FIXED64 = 1;
constexpr uint32_t WIRE_LENGTH_DELIMITED = 2;
constexpr uint32_t WIRE_FIXED32 = 5;

// onnx.proto ModelProto field numbers
constexpr uint32_t MODEL_PRODUCER_NAME = 2;
constexpr uint32_t MODEL_PRODUCER_VERSION = 3;
constexpr uint32_t MODEL_DOMAIN = 4;
constexpr uint32_t MODEL_MODEL_VERSION = 5;
constexpr uint32_t MODEL_DOC_STRING = 6;
constexpr uint32_t MODEL_METADATA_PROPS = 14;

// onnx.proto StringStringEntryProto field numbers
constexpr uint32_t ENTRY_KEY = 1;
constexpr uint32_t ENTRY_VALUE = 2;

/// Sequential reader over either an istream or an in-memory buffer.
class WireReader
{
public:
  explicit WireReader(std::istream & stream) : stream_(&stream) {}
  explicit WireReader(const std::string & buffer) : buffer_(&buffer) {}

  bool at_end()
  {
    if (buffer_) return pos_ >= buffer_->size();
    return stream_->peek() == std::char_traits<char>::eof();
  }

  uint64_t read_varint()
  {
    uint64_t result = 0;
    for (int shift = 0; shift < 64; shift += 7) {
      const auto byte = static_cast<uint8_t>(read_byte());
      result |= static_cast<uint64_t>(byte & 0x7F) << shift;
      if ((byte & 0x80) == 0) return result;
    }
    throw std::runtime_error("Malformed varint");
  }

  std::string read_bytes(const uint64_t size)
  {
    if (buffer_) {
      if (size > buffer_->size() - pos_) throw std::runtime_error("Unexpected end of data");
      std::string out = buffer_->substr(pos_, size);
      pos_ += size;
      return out;
    }
    std::string out(size, '\0');
    if (!stream_->read(out.data(), static_cast<std::streamsize>(size))) {
      throw std::runtime_error("Unexpected end of file");
    }
    return out;
  }

  void skip(const uint64_t size)
  {
    if (buffer_) {
      if (size > buffer_->size() - pos_) throw std::runtime_error("Unexpected end of data");
      pos_ += size;
      return;
    }
    stream_->seekg(static_cast<std::streamoff>(size), std::ios::cur);
    if (!*stream_) throw std::runtime_error("Unexpected end of file");
  }

  void skip_field(const uint32_t wire_type)
  {
    switch (wire_type) {
      case WIRE_VARINT:
        read_varint();
        break;
      case WIRE_FIXED64:
        skip(8);
        break;
      case WIRE_LENGTH_DELIMITED:
        skip(read_varint());
        break;
      case WIRE_FIXED32:
        skip(4);
        break;
      default:
        throw std::runtime_error("Unsupported wire type " + std::to_string(wire_type));
    }
  }

private:
  char read_byte()
  {
    if (buffer_) {
      if (pos_ >= buffer_->size()) throw std::runtime_error("Unexpected end of data");
      return (*buffer_)[pos_++];
    }
    char c{};
    if (!stream_->get(c)) throw std::runtime_error("Unexpected end of file");
    return c;
  }

  std::istream * stream_{nullptr};
  const std::string * buffer_{nullptr};
  size_t pos_{0};
};

std::pair<std::string, std::string> parse_string_string_entry(const std::string & bytes)
{
  WireReader reader(bytes);
  std::string key;
  std::string value;
  while (!reader.at_end()) {
    const uint64_t tag = reader.read_varint();
    const auto field = static_cast<uint32_t>(tag >> 3);
    const auto wire_type = static_cast<uint32_t>(tag & 0x7);
    if (wire_type == WIRE_LENGTH_DELIMITED && field == ENTRY_KEY) {
      key = reader.read_bytes(reader.read_varint());
    } else if (wire_type == WIRE_LENGTH_DELIMITED && field == ENTRY_VALUE) {
      value = reader.read_bytes(reader.read_varint());
    } else {
      reader.skip_field(wire_type);
    }
  }
  return {std::move(key), std::move(value)};
}
}  // namespace

ModelVersion ModelVersion::decode(const int64_t encoded)
{
  constexpr int64_t field_base = 1'000'000;
  return {
    encoded / (field_base * field_base), (encoded / field_base) % field_base, encoded % field_base};
}

std::string ModelVersion::to_string() const
{
  return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
}

std::optional<std::string> OnnxModelMetadata::get(const std::string & key) const
{
  const auto it = metadata_props.find(key);
  if (it == metadata_props.end()) return std::nullopt;
  return it->second;
}

OnnxModelMetadata read_onnx_metadata(const std::string & model_path)
{
  std::ifstream file(model_path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("Failed to open ONNX model: " + model_path);
  }

  OnnxModelMetadata metadata;
  WireReader reader(file);
  try {
    while (!reader.at_end()) {
      const uint64_t tag = reader.read_varint();
      const auto field = static_cast<uint32_t>(tag >> 3);
      const auto wire_type = static_cast<uint32_t>(tag & 0x7);

      if (wire_type == WIRE_VARINT && field == MODEL_MODEL_VERSION) {
        metadata.model_version = static_cast<int64_t>(reader.read_varint());
        continue;
      }
      if (wire_type != WIRE_LENGTH_DELIMITED) {
        reader.skip_field(wire_type);
        continue;
      }

      const uint64_t size = reader.read_varint();
      switch (field) {
        case MODEL_PRODUCER_NAME:
          metadata.producer_name = reader.read_bytes(size);
          break;
        case MODEL_PRODUCER_VERSION:
          metadata.producer_version = reader.read_bytes(size);
          break;
        case MODEL_DOMAIN:
          metadata.domain = reader.read_bytes(size);
          break;
        case MODEL_DOC_STRING:
          metadata.doc_string = reader.read_bytes(size);
          break;
        case MODEL_METADATA_PROPS: {
          auto [key, value] = parse_string_string_entry(reader.read_bytes(size));
          metadata.metadata_props[std::move(key)] = std::move(value);
          break;
        }
        default:
          // graph, opset_import, functions, ... are skipped without being loaded.
          reader.skip(size);
          break;
      }
    }
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Failed to parse ONNX metadata from " + model_path + ": " + e.what());
  }
  return metadata;
}

}  // namespace autoware::ml_planner::utils
