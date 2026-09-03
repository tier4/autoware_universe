cmake_minimum_required(VERSION 3.14)
project(autoware_tensorrt_e2e)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Options
option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

# Find CUDA
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if(CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if(CUDA_VERBOSE)
    message(STATUS "CUDA is available!")
    message(STATUS "CUDA Libs: ${CUDA_LIBRARIES}")
    message(STATUS "CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif()
  unset(CUDA_cublas_device_LIBRARY CACHE)
  set(CUDA_AVAIL ON)
else()
  message(WARNING "CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# Find TensorRT libraries
option(TRT_AVAIL "TensorRT available" OFF)
find_library(NVINFER nvinfer)
find_library(NVONNXPARSER nvonnxparser)
if(NVINFER AND NVONNXPARSER)
  if(CUDA_VERBOSE)
    message(STATUS "TensorRT is available!")
    message(STATUS "NVINFER: ${NVINFER}")
    message(STATUS "NVONNXPARSER: ${NVONNXPARSER}")
  endif()
  set(TRT_AVAIL ON)
else()
  message(WARNING "TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

if(TRT_AVAIL AND CUDA_AVAIL)
  # Suppress warnings originating from TensorRT headers
  add_compile_options(-Wno-deprecated-declarations)

  find_package(OpenCV REQUIRED)

  include_directories(
    include
    ${CUDA_INCLUDE_DIRS}
  )

  # CUDA architecture settings (aligned with autoware_tensorrt_vad)
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_75,code=sm_75")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_86,code=sm_86")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_87,code=sm_87")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=sm_89")
  if(CUDA_VERSION VERSION_GREATER_EQUAL "12.8")
    if(CUDA_VERSION VERSION_LESS "13.0")
      list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_101,code=sm_101")
    else()  # CUDA 13.0 renamed SM101 to SM110
      list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_110,code=sm_110")
    endif()
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_120,code=sm_120")
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_120,code=compute_120")
  else()
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=compute_89")
  endif()

  cuda_add_library(${PROJECT_NAME}_cuda_lib SHARED
    src/preprocess/camera_preprocess_kernel.cu
    src/bev_feature/bev_warp_kernel.cu
  )

  ament_auto_add_library(${PROJECT_NAME}_component SHARED
    src/types.cpp
    src/input_provider_registry.cpp
    src/inference_engine.cpp
    src/bev_feature/temporal_bev_cache.cpp
    src/bev_feature/trt_bev_feature_extractor.cpp
    src/providers/camera_input_provider.cpp
    src/providers/lidar_input_provider.cpp
    src/providers/context_input_provider.cpp
    src/providers/bev_feature_input_provider.cpp
    src/postprocess/trajectory_postprocessor.cpp
    src/tensorrt_e2e_node.cpp
  )

  # The BEVFusion voxelization kernels live in a separately installed CUDA library that
  # autoware_bevfusion does not export as an ament target.
  find_library(AUTOWARE_BEVFUSION_CUDA_LIB autoware_bevfusion_cuda_lib
    HINTS ${autoware_bevfusion_DIR}/../../../lib
    REQUIRED
  )

  target_link_libraries(${PROJECT_NAME}_component
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${OpenCV_LIBS}
    ${PROJECT_NAME}_cuda_lib
    ${AUTOWARE_BEVFUSION_CUDA_LIB}
  )

  rclcpp_components_register_node(${PROJECT_NAME}_component
    PLUGIN "autoware::tensorrt_e2e::TensorrtE2eNode"
    EXECUTABLE ${PROJECT_NAME}_node
  )

  install(
    TARGETS ${PROJECT_NAME}_cuda_lib
    DESTINATION lib
  )

  if(BUILD_TESTING)
    ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
      test/types_test.cpp
      test/trajectory_postprocessor_test.cpp
      test/bev_warp_test.cpp
      test/bev_warp_golden_test.cpp
      test/temporal_bev_cache_test.cpp
    )
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME}_component)
    target_compile_definitions(test_${PROJECT_NAME} PRIVATE
      TENSORRT_E2E_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  endif()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  urdf
)
