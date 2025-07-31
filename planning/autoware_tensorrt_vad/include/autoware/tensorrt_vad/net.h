/*
 * SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef NET_H_
#define NET_H_

#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <unordered_map>

#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include "tensor.h"

#include <autoware/tensorrt_common/tensorrt_common.hpp>

namespace nv {
using namespace nvinfer1;

struct Net {
  ICudaEngine* engine;
  IExecutionContext* context;
  nv::TensorMap bindings;
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> trt_common;

  Net(
    IRuntime* runtime, 
    nv::TensorMap& ext,
    std::unique_ptr<autoware::tensorrt_common::TrtCommon> trt_common_to_own
  ): trt_common{std::move(trt_common_to_own)}
  {
    std::string engine_path = trt_common->getTrtCommonConfig()->engine_path.string();
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file) {
      throw std::runtime_error("Error opening engine file: " + engine_path);
    }
    engine_file.seekg(0, engine_file.end);
    long int fsize = engine_file.tellg();
    engine_file.seekg(0, engine_file.beg);

    // Read the engine file into a buffer
    std::vector<char> engineData(fsize);

    engine_file.read(engineData.data(), fsize);
    engine = runtime->deserializeCudaEngine(engineData.data(), fsize);
    context = engine->createExecutionContext(); 

    int32_t nb = trt_common->getNbIOTensors();

    for (int32_t n = 0; n < nb; n++) {
      std::string name = trt_common->getIOTensorName(n);
      Dims d = trt_common->getTensorShape(name.c_str());
      DataType dtype = engine->getTensorDataType(name.c_str());
      
      if (ext.find(name) != ext.end()) {
        // use external memory
        trt_common->setTensorAddress(name.c_str(), ext[name]->ptr);
      } else {
        bindings[name] = std::make_shared<Tensor>(name, d, dtype);
        trt_common->setTensorAddress(name.c_str(), bindings[name]->ptr);
      }
    }
  }

  void Enqueue(cudaStream_t stream) {
    trt_common->enqueueV3(stream);
  }

  ~Net() {
      if (context) {
          context->destroy();
          context = nullptr;
      }
      if (engine) {
          engine->destroy();
          engine = nullptr;
      }
      
      // bindingsの各Tensorのメモリを解放
      for (auto& pair : bindings) {
          pair.second.reset();
      }
      bindings.clear();
  }
}; // class Net

} // namespace nv

#endif // NET_H_
