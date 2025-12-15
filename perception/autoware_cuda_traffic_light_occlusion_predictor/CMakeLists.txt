cmake_minimum_required(VERSION 3.14)
project(autoware_cuda_traffic_light_occlusion_predictor)

# Find CUDA
find_package(ament_cmake_auto REQUIRED)
find_package(CUDA)

if(NOT ${CUDA_FOUND})
  message(WARNING "CUDA was not found, so the autoware_cuda_traffic_light_occlusion_predictor package will not be built.")
  return()
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CUDA_FLAGS ${CMAKE_CUDA_FLAGS} "-g -G")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-g -G")
endif()

ament_auto_find_build_dependencies()

# Find additional dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()

if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
  SYSTEM
  ${CUDA_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# CUDA compilation flags
# cSpell: ignore expt gencode
list(APPEND CUDA_NVCC_FLAGS "--expt-relaxed-constexpr -diag-suppress 20012")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_75,code=sm_75")  # Turing
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_86,code=sm_86")  # Ampere
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_87,code=sm_87")  # Ampere (Orin)
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=sm_89")  # Ada Lovelace
# PTX support for newer GPUs
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=compute_89")

# __host__ or __device__ annotation on lambda requires --extended-lambda nvcc flag
list(APPEND CUDA_NVCC_FLAGS "--extended-lambda")

# cSpell: ignore fmad
# NOTE: adding `--fmad=false` can reduce numerical discrepancies between CPU and GPU
# implementations, but introduces slight performance impact. Disabled for performance.
# list(APPEND CUDA_NVCC_FLAGS "--fmad=false")

################################################################################
### CUDA Library ###
cuda_add_library(cuda_traffic_light_occlusion_predictor_lib SHARED
  src/cuda_occlusion_predictor/cuda_occlusion_predictor.cu
  src/cuda_occlusion_predictor/cuda_occlusion_kernels.cu
)

target_link_libraries(cuda_traffic_light_occlusion_predictor_lib
  ${PCL_LIBRARIES}
)

target_include_directories(cuda_traffic_light_occlusion_predictor_lib SYSTEM PRIVATE
  ${autoware_cuda_utils_INCLUDE_DIRS}
  ${cuda_blackboard_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

################################################################################
### ROS2 Node ###
ament_auto_add_library(cuda_traffic_light_occlusion_predictor SHARED
  src/cuda_traffic_light_occlusion_predictor_node.cpp
)

target_include_directories(cuda_traffic_light_occlusion_predictor SYSTEM PRIVATE
  ${autoware_lanelet2_extension_INCLUDE_DIRS}
  ${autoware_map_msgs_INCLUDE_DIRS}
  ${autoware_traffic_light_utils_INCLUDE_DIRS}
  ${autoware_utils_INCLUDE_DIRS}
  ${cuda_blackboard_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${geometry_msgs_INCLUDE_DIRS}
  ${image_geometry_INCLUDE_DIRS}
  ${perception_utils_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tier4_perception_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

target_link_libraries(cuda_traffic_light_occlusion_predictor
  ${CUDA_LIBRARIES}
  cuda_traffic_light_occlusion_predictor_lib
)

# Register node component
rclcpp_components_register_node(cuda_traffic_light_occlusion_predictor
  PLUGIN "autoware::traffic_light::CudaTrafficLightOcclusionPredictorNode"
  EXECUTABLE cuda_traffic_light_occlusion_predictor_node
)

################################################################################
# Install
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS cuda_traffic_light_occlusion_predictor_lib
  LIBRARY DESTINATION lib
)

# ============================================================================
# Testing
# ============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit test for CUDA kernels
  cuda_add_executable(test_cuda_occlusion_kernels
    test/test_cuda_occlusion_kernels.cpp
    src/cuda_occlusion_predictor/cuda_occlusion_kernels.cu
  )
  target_include_directories(test_cuda_occlusion_kernels PRIVATE
    ${CUDA_INCLUDE_DIRS}
    ${autoware_cuda_utils_INCLUDE_DIRS}
  )
  target_link_libraries(test_cuda_occlusion_kernels
    GTest::GTest
    GTest::Main
  )

  # Unit test for CUDA predictor
  ament_add_gtest(test_cuda_occlusion_predictor
    test/test_cuda_occlusion_predictor.cpp
  )
  if(TARGET test_cuda_occlusion_predictor)
    ament_target_dependencies(test_cuda_occlusion_predictor
      autoware_cuda_utils
      cuda_blackboard
      sensor_msgs
    )
    target_link_libraries(test_cuda_occlusion_predictor
      cuda_traffic_light_occlusion_predictor_lib
    )
  endif()

  # Integration test for node
  ament_add_gtest(test_node_integration
    test/test_node_integration.cpp
  )
  if(TARGET test_node_integration)
    target_link_libraries(test_node_integration
      cuda_traffic_light_occlusion_predictor
    )
  endif()

endif()

ament_auto_package()

# Set ROS_DISTRO macros
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STREQUAL "rolling")
  add_compile_definitions(ROS_DISTRO_ROLLING)
elseif(${ROS_DISTRO} STREQUAL "foxy")
  add_compile_definitions(ROS_DISTRO_FOXY)
elseif(${ROS_DISTRO} STREQUAL "galactic")
  add_compile_definitions(ROS_DISTRO_GALACTIC)
elseif(${ROS_DISTRO} STREQUAL "humble")
  add_compile_definitions(ROS_DISTRO_HUMBLE)
endif()

