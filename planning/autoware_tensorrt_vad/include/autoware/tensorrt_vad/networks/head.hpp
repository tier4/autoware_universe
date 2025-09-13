// Copyright 2025 Shin-kyoto.
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

#ifndef AUTOWARE_TENSORRT_VAD_HEAD_HPP_
#define AUTOWARE_TENSORRT_VAD_HEAD_HPP_

#include "autoware/tensorrt_vad/networks/net.hpp"

namespace autoware::tensorrt_vad {

class Head : public Net {
private:
  NetworkType network_type_;

public:
  Head(
    const VadConfig& vad_config,
    const autoware::tensorrt_common::TrtCommonConfig& trt_common_config,
    NetworkType network_type,
    const std::string& plugins_path,
    std::shared_ptr<VadLogger> logger
  );

  std::vector<autoware::tensorrt_common::NetworkIO> generate_network_io(const VadConfig& vad_config) override;
  void set_input_tensor(TensorMap& ext) override;
};

} // namespace autoware::tensorrt_vad

#endif // AUTOWARE_TENSORRT_VAD_HEAD_HPP_
