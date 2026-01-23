// Copyright 2024 TIER IV, Inc.
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
/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/*
 * This code is licensed under CC0 1.0 Universal (Public Domain).
 * You can use this without any limitation.
 * https://creativecommons.org/publicdomain/zero/1.0/deed.en
 */

#ifndef AUTOWARE__LIDAR_TRANSFUSION__CUDA_UTILS_HPP_
#define AUTOWARE__LIDAR_TRANSFUSION__CUDA_UTILS_HPP_

#include <cuda_runtime_api.h>

#include <memory>
#include <sstream>
#include <stdexcept>
#include <type_traits>

#define CHECK_CUDA_ERROR(e) (cuda::check_error(e, __FILE__, __LINE__))

namespace cuda
{
inline void check_error(const ::cudaError_t e, const char * f, int n)
{
  if (e != ::cudaSuccess) {
    ::std::stringstream s;
    s << ::cudaGetErrorName(e) << " (" << e << ")@" << f << "#L" << n << ": "
      << ::cudaGetErrorString(e);
    throw ::std::runtime_error{s.str()};
  }
}

struct deleter
{
  void operator()(void * p) const { CHECK_CUDA_ERROR(::cudaFree(p)); }
};

template <typename T>
using unique_ptr = ::std::unique_ptr<T, deleter>;

template <typename T>
typename ::std::enable_if<::std::is_array<T>::value, cuda::unique_ptr<T>>::type make_unique(
  const ::std::size_t n)
{
  using U = typename ::std::remove_extent<T>::type;
  U * p;
  CHECK_CUDA_ERROR(::cudaMalloc(reinterpret_cast<void **>(&p), sizeof(U) * n));
  return cuda::unique_ptr<T>{p};
}

template <typename T>
cuda::unique_ptr<T> make_unique()
{
  T * p;
  CHECK_CUDA_ERROR(::cudaMalloc(reinterpret_cast<void **>(&p), sizeof(T)));
  return cuda::unique_ptr<T>{p};
}

// #include <iostream>
// #include <memory>
// #include <stdexcept>
// #include <cuda_runtime.h>

/**
 * @brief 非同期解放を行うカスタムデリータ
 */
struct CudaAsyncDeleter {
    cudaStream_t stream = nullptr;

    CudaAsyncDeleter() = default;
/*
    CudaAsyncDeleter() = delete;
    CudaAsyncDeleter(const CudaAsyncDeleter&) = delete;
    CudaAsyncDeleter& operator=(const CudaAsyncDeleter&) = delete;
    CudaAsyncDeleter(CudaAsyncDeleter&&) noexcept = default;
    CudaAsyncDeleter& operator=(CudaAsyncDeleter&&) noexcept = default;
*/

    explicit CudaAsyncDeleter(cudaStream_t s) : stream(s) {}

    void operator()(void* ptr) const {
        if (ptr) {
            // cudaFreeAsync は単一オブジェクトでも配列でも共通
            cudaFreeAsync(ptr, stream);
        }
    }
};

/**
 * @brief デバイス非同期メモリ管理用 unique_ptr
 */
template <typename T>
using async_unique_ptr = std::unique_ptr<T, CudaAsyncDeleter>;

/**
 * @brief 単一オブジェクト用 (make_unique_async<T>(stream))
 */
template <typename T>
typename std::enable_if<!std::is_array<T>::value, async_unique_ptr<T>>::type
make_unique_async(cudaStream_t stream) {
    T* raw_ptr = nullptr;
    cudaError_t err = cudaMallocAsync(reinterpret_cast<void**>(&raw_ptr), sizeof(T), stream);

    if (err != cudaSuccess) {
        throw std::runtime_error(std::string("CUDA Alloc Failed: ") + cudaGetErrorString(err));
    }
    return async_unique_ptr<T>(raw_ptr, CudaAsyncDeleter(stream));
}

/**
 * @brief 配列用 (make_unique_async<T[]>(count, stream))
 */
template <typename T>
typename std::enable_if<std::is_array<T>::value, async_unique_ptr<T>>::type
make_unique_async(size_t count, cudaStream_t stream) {
    using ElementType = typename std::remove_extent<T>::type;
    ElementType* raw_ptr = nullptr;

    // 要素数 * 型サイズ で確保
    cudaError_t err = cudaMallocAsync(reinterpret_cast<void**>(&raw_ptr), sizeof(ElementType) * count, stream);

    if (err != cudaSuccess) {
        throw std::runtime_error(std::string("CUDA Array Alloc Failed: ") + cudaGetErrorString(err));
    }
    return async_unique_ptr<T>(raw_ptr, CudaAsyncDeleter(stream));
}

// --- 使用例 ---
/*
int main() {
    cudaStream_t stream;
    cudaStreamCreate(&stream);

    try {
        // 1. 単一オブジェクトの確保
        auto d_single = make_unique_async<float>(stream);

        // 2. 配列の確保 (例: float[100])
        size_t n = 100;
        auto d_array = make_unique_async<float[]>(n, stream);

        // d_array[5] = 1.0f; // ホストからのアクセスは不可 (Device Memoryのため)

    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }

    cudaStreamSynchronize(stream);
    cudaStreamDestroy(stream);
    return 0;
}
*/

constexpr std::size_t CUDA_ALIGN = 256;

template <typename T>
inline std::size_t get_size_aligned(size_t num_elem)
{
  std::size_t size = num_elem * sizeof(T);
  std::size_t extra_align = 0;
  if (size % CUDA_ALIGN != 0) {
    extra_align = CUDA_ALIGN - size % CUDA_ALIGN;
  }
  return size + extra_align;
}

template <typename T>
inline T * get_next_ptr(size_t num_elem, void *& workspace, std::size_t & workspace_size)
{
  std::size_t size = get_size_aligned<T>(num_elem);
  if (size > workspace_size) {
    throw ::std::runtime_error("Workspace is too small!");
  }
  workspace_size -= size;
  T * ptr = reinterpret_cast<T *>(workspace);
  workspace = reinterpret_cast<void *>(reinterpret_cast<uintptr_t>(workspace) + size);
  return ptr;
}

template <typename T>
void clear_async(T * ptr, std::size_t num_elem, cudaStream_t stream)
{
  CHECK_CUDA_ERROR(::cudaMemsetAsync(ptr, 0, sizeof(T) * num_elem, stream));
}

}  // namespace cuda

#endif  // AUTOWARE__LIDAR_TRANSFUSION__CUDA_UTILS_HPP_
