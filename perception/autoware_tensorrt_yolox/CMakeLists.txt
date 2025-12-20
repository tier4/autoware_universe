cmake_minimum_required(VERSION 3.17)
project(autoware_tensorrt_yolox)

find_package(autoware_tensorrt_common)
if(NOT ${autoware_tensorrt_common_FOUND})
  message(WARNING "The autoware_tensorrt_common package is not found. Please check its dependencies.")
  return()
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

# Debug output
message(WARNING "=== DEBUG autoware_tensorrt_yolox ===")

# Check CMAKE paths
message(WARNING "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

# Try to find the package and see what files exist
find_package(autoware_object_recognition_utils QUIET)
if(autoware_object_recognition_utils_FOUND)
  message(WARNING "Package FOUND")
  message(WARNING "  _DIR: ${autoware_object_recognition_utils_DIR}")
  
  # Check what files exist in that directory
  if(EXISTS "${autoware_object_recognition_utils_DIR}")
    file(GLOB config_files "${autoware_object_recognition_utils_DIR}/*.cmake")
    message(WARNING "  Config files found:")
    foreach(config_file ${config_files})
      get_filename_component(filename ${config_file} NAME)
      message(WARNING "    - ${filename}")
    endforeach()
    
    # Read the main config file
    set(main_config "${autoware_object_recognition_utils_DIR}/autoware_object_recognition_utilsConfig.cmake")
    if(EXISTS "${main_config}")
      file(READ "${main_config}" config_content)
      message(WARNING "  Main config content (first 1000 chars):")
      string(SUBSTRING "${config_content}" 0 1000 config_preview)
      message(WARNING "${config_preview}")
    endif()
    
    # Check for targets file
    set(targets_file "${autoware_object_recognition_utils_DIR}/export_autoware_object_recognition_utilsTargets.cmake")
    if(EXISTS "${targets_file}")
      message(WARNING "  Targets file EXISTS:  ${targets_file}")
      file(READ "${targets_file}" targets_content)
      string(SUBSTRING "${targets_content}" 0 500 targets_preview)
      message(WARNING "  Targets file preview:  ${targets_preview}")
    else()
      message(WARNING "  Targets file DOES NOT EXIST:  ${targets_file}")
      # List all Targets files
      file(GLOB targets_files "${autoware_object_recognition_utils_DIR}/*Targets.cmake")
      message(WARNING "  All *Targets.cmake files:")
      foreach(tf ${targets_files})
        message(WARNING "    - ${tf}")
      endforeach()
    endif()
  endif()
  
  # Check if target exists NOW
  if(TARGET autoware_object_recognition_utils)
    message(WARNING "  Target EXISTS after find_package")
    get_target_property(target_type autoware_object_recognition_utils TYPE)
    message(WARNING "    Type: ${target_type}")
    get_target_property(interface_libs autoware_object_recognition_utils INTERFACE_LINK_LIBRARIES)
    message(WARNING "    INTERFACE_LINK_LIBRARIES:  ${interface_libs}")
  else()
    message(WARNING "  Target DOES NOT EXIST after find_package")
  endif()
else()
  message(WARNING "Package NOT FOUND")
endif()

message(WARNING "=== END DEBUG ===")

# TODO(amadeuszsz): Remove -Wno-deprecated-declarations once removing implicit quantization
add_compile_options(-Wno-deprecated-declarations)

find_package(OpenCV REQUIRED)

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if(CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if(CUDA_VERBOSE)
    message("CUDA is available!")
    message("CUDA Libs: ${CUDA_LIBRARIES}")
    message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif()
  # Note: cublas_device was depreciated in CUDA version 9.2
  #       https://forums.developer.nvidia.com/t/where-can-i-find-libcublas-device-so-or-libcublas-device-a/67251/4
  #       In LibTorch, CUDA_cublas_device_LIBRARY is used.
  unset(CUDA_cublas_device_LIBRARY CACHE)
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER nvinfer)
find_library(NVONNXPARSER nvonnxparser)
if(NVINFER AND NVONNXPARSER)
  if(CUDA_VERBOSE)
    message("TensorRT is available!")
    message("NVINFER: ${NVINFER}")
    message("NVONNXPARSER: ${NVONNXPARSER}")
  endif()
  set(TRT_AVAIL ON)
else()
  message("TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

# set flags for CUDNN availability
option(CUDNN_AVAIL "CUDNN available" OFF)
# try to find the CUDNN module
find_library(CUDNN_LIBRARY
NAMES libcudnn.so${__cudnn_ver_suffix} libcudnn${__cudnn_ver_suffix}.dylib ${__cudnn_lib_win_name}
PATHS $ENV{LD_LIBRARY_PATH} ${__libpath_cudart} ${CUDNN_ROOT_DIR} ${PC_CUDNN_LIBRARY_DIRS} ${CMAKE_INSTALL_PREFIX}
PATH_SUFFIXES lib lib64 bin
DOC "CUDNN library."
)
if(CUDNN_LIBRARY)
  if(CUDA_VERBOSE)
    message(STATUS "CUDNN is available!")
    message(STATUS "CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  endif()
  set(CUDNN_AVAIL ON)
else()
  message("CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

find_package(OpenMP)
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

##########
# tensorrt_yolox
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/tensorrt_yolox.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  OpenCV
)

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  # Officially, add_library supports .cu file compilation.
  # However, as of cmake 3.22.1, it seems to fail compilation because compiler flags for
  # C++ are directly passed to nvcc (they are originally space separated
  # but nvcc assume comma separated as argument of `-Xcompiler` option).
  # That is why `cuda_add_library` is used here.

  # cSpell: ignore gencode
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_75,code=sm_75")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_86,code=sm_86")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_87,code=sm_87")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=sm_89")
  # NOTE(knzo25): PTX support for newer GPUs until we can compile directly
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=compute_89")
  # TODO(knzo25): enable when the driver supports it
  # list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_120,code=sm_120")

  cuda_add_library(${PROJECT_NAME}_gpu_preprocess
    SHARED
    src/preprocess.cu
  )

  target_include_directories(${PROJECT_NAME}_gpu_preprocess PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

  target_link_libraries(${PROJECT_NAME}
    ${autoware_tensorrt_common_LIBRARIES}
    ${PROJECT_NAME}_gpu_preprocess
  )

  install(
    TARGETS ${PROJECT_NAME}_gpu_preprocess
    LIBRARY DESTINATION lib
  )
else()
  target_link_libraries(${PROJECT_NAME}
    ${autoware_tensorrt_common_LIBRARIES}
  )
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

ament_auto_add_library(yolox_single_image_inference_node SHARED
  src/yolox_single_image_inference_node.cpp
)

ament_target_dependencies(yolox_single_image_inference_node
  OpenCV
)

target_link_libraries(yolox_single_image_inference_node
  ${PROJECT_NAME}
  stdc++fs
)

target_compile_definitions(yolox_single_image_inference_node PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

rclcpp_components_register_node(yolox_single_image_inference_node
  PLUGIN "autoware::tensorrt_yolox::YoloXSingleImageInferenceNode"
  EXECUTABLE yolox_single_image_inference
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/tensorrt_yolox_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  OpenCV
  tf2_ros
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

target_compile_definitions(${PROJECT_NAME}_node PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::tensorrt_yolox::TrtYoloXNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
