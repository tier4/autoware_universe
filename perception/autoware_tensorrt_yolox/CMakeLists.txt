cmake_minimum_required(VERSION 3.17)
project(autoware_tensorrt_yolox)

find_package(autoware_tensorrt_common)
if(NOT ${autoware_tensorrt_common_FOUND})
  message(WARNING "The autoware_tensorrt_common package is not found. Please check its dependencies.")
  return()
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

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


# =================================================
set(JETSON_MM_API_PATH "/usr/src/jetson_multimedia_api")
if(NOT EXISTS "${JETSON_MM_API_PATH}")
  message(FATAL_ERROR "Jetson Multimedia API not found. Please install: sudo apt install nvidia-l4t-jetson-multimedia-api")
endif()

set(PATCHED_SRC_DIR "${CMAKE_CURRENT_BINARY_DIR}/jetson_patched")
file(MAKE_DIRECTORY ${PATCHED_SRC_DIR})

set(JETSON_HEADERS 
    "NvJpegDecoder.h"
    "NvElement.h" 
    "NvElementProfiler.h"
    "NvLogging.h"
)

set(JETSON_SOURCES 
    "NvJpegDecoder.cpp" 
    "NvElement.cpp" 
    "NvElementProfiler.cpp"
    "NvLogging.cpp"
)

set(JP6_JPEG_HEADER "${JETSON_MM_API_PATH}/include/libjpeg-8b/jpeglib.h")
if(NOT EXISTS "${JP6_JPEG_HEADER}")
  message(FATAL_ERROR "Could not find jpeglib.h at ${JP6_JPEG_HEADER}. Please check your Jetson Multimedia API installation.")
endif()

foreach(fname ${JETSON_HEADERS})
  set(FULL_PATH "${JETSON_MM_API_PATH}/include/${fname}")
  if(EXISTS ${FULL_PATH})
    file(READ "${FULL_PATH}" FILE_CONTENT)
    string(REPLACE "#include <jpeglib.h>" "#include \"${JP6_JPEG_HEADER}\"" FILE_CONTENT "${FILE_CONTENT}")
    string(REPLACE "#include \"jpeglib.h\"" "#include \"${JP6_JPEG_HEADER}\"" FILE_CONTENT "${FILE_CONTENT}")
    file(WRITE "${PATCHED_SRC_DIR}/${fname}" "${FILE_CONTENT}")
  else()
    message(FATAL_ERROR "Could not find header file: ${FULL_PATH}")
  endif()
endforeach()

foreach(fname ${JETSON_SOURCES})
  set(FULL_PATH "${JETSON_MM_API_PATH}/samples/common/classes/${fname}")
  if(EXISTS ${FULL_PATH})
    file(READ "${FULL_PATH}" FILE_CONTENT)
    string(REPLACE "#include <jpeglib.h>" "#include \"${JP6_JPEG_HEADER}\"" FILE_CONTENT "${FILE_CONTENT}")
    string(REPLACE "#include \"jpeglib.h\"" "#include \"${JP6_JPEG_HEADER}\"" FILE_CONTENT "${FILE_CONTENT}")
    file(WRITE "${PATCHED_SRC_DIR}/${fname}" "${FILE_CONTENT}")
  else()
    message(FATAL_ERROR "Could not find source file: ${FULL_PATH}")
  endif()
endforeach()

set(CUDA_LIB_PATHS
    /usr/local/cuda/lib64
    /usr/lib/aarch64-linux-gnu/nvidia/
    /usr/lib/aarch64-linux-gnu/tegra/
)

find_library(NVJPEG_LIBRARY nvjpeg PATHS ${CUDA_LIB_PATHS})
find_library(NVBUF_SURFACE_LIB nvbufsurface PATHS ${CUDA_LIB_PATHS})
find_library(NVBUF_TRANSFORM_LIB nvbufsurftransform PATHS ${CUDA_LIB_PATHS})

if(NOT NVBUF_SURFACE_LIB OR NOT NVBUF_TRANSFORM_LIB)
  message(FATAL_ERROR "NvBufSurface libraries not found. Ensure you are on JetPack 6.")
endif()
# =================================================

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
  ${PATCHED_SRC_DIR}/NvJpegDecoder.cpp
  ${PATCHED_SRC_DIR}/NvElement.cpp
  ${PATCHED_SRC_DIR}/NvElementProfiler.cpp
  ${PATCHED_SRC_DIR}/NvLogging.cpp
)

target_include_directories(${PROJECT_NAME} BEFORE PUBLIC
  ${PATCHED_SRC_DIR}             
  ${JETSON_MM_API_PATH}/include/libjpeg-8b
  ${JETSON_MM_API_PATH}/include
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${NVJPEG_LIBRARY}
  ${CUDA_LIBRARIES}
  ${NVBUF_SURFACE_LIB}
  ${NVBUF_TRANSFORM_LIB}
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
