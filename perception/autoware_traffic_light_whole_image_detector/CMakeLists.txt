cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_whole_image_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

# CUDA
option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)
find_package(CUDA QUIET)
if(CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64 ${CUDA_TOOLKIT_ROOT_DIR}/lib)
  set(CUDA_AVAIL ON)
else()
  set(CUDA_AVAIL OFF)
endif()

# TensorRT
find_library(NVINFER NAMES nvinfer QUIET)
find_library(NVONNXPARSER nvonnxparser QUIET)
find_library(NVINFER_PLUGIN NAMES nvinfer_plugin QUIET)
if(NVINFER AND NVONNXPARSER AND NVINFER_PLUGIN)
  set(TRT_AVAIL ON)
else()
  set(TRT_AVAIL OFF)
endif()

# cuDNN
find_library(CUDNN_LIBRARY NAMES libcudnn.so cudnn
  PATHS $ENV{LD_LIBRARY_PATH} $ENV{CUDNN_ROOT_DIR}
  PATH_SUFFIXES lib lib64 DOC "cuDNN library")
if(CUDNN_LIBRARY)
  set(CUDNN_AVAIL ON)
else()
  set(CUDNN_AVAIL OFF)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  add_definitions(-DENABLE_GPU)
  include_directories(${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})

  ament_auto_add_library(${PROJECT_NAME} SHARED
    src/whole_image_detector_node.cpp
    src/lightnet_decoder.cpp
    src/comlops_detector.cpp
  )
  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}
    ament_index_cpp autoware_cuda_utils autoware_internal_debug_msgs autoware_perception_msgs
    autoware_tensorrt_common autoware_utils cv_bridge rclcpp sensor_msgs tier4_perception_msgs
  )
  target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    ${NVINFER} ${NVONNXPARSER} ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES} ${CUBLAS_LIBRARIES} ${CUDNN_LIBRARY}
    stdc++fs
  )
  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "autoware::traffic_light::WholeImageDetectorNode"
    EXECUTABLE traffic_light_whole_image_detector_node
  )
  message(STATUS "TensorRT found: building whole_image_detector with inference (ComlopsDetector)")
else()
  message(STATUS "TensorRT/CUDA/cuDNN not all available; building stub (empty DetectedObjectsWithFeature)")
  ament_auto_add_library(${PROJECT_NAME} SHARED src/whole_image_detector_node_stub.cpp)
  ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs tier4_perception_msgs)
  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "autoware::traffic_light::WholeImageDetectorNode"
    EXECUTABLE traffic_light_whole_image_detector_node
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
