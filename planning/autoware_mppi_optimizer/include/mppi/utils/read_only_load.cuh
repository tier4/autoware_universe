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

#include <cuda_runtime.h>

namespace mppi::memory
{

/**
 * Load immutable cycle data through CUDA's read-only load path.
 *
 * The host implementation is an ordinary dereference so cost geometry code can remain shared
 * between CPU validation and GPU rollout evaluation.
 */
template <class T>
__host__ __device__ __forceinline__ T loadReadOnly(const T * address)
{
#ifdef __CUDA_ARCH__
  return __ldg(address);
#else
  return *address;
#endif
}

}  // namespace mppi::memory
