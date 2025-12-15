cmake_minimum_required(VERSION 3.14)
project(autoware_image_transport_decompressor)

find_package(autoware_cmake REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

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

if(NOT NVBUF_SURFACE_LIB)
  message(FATAL_ERROR "NvBufSurface libraries not found. Ensure you are on JetPack 6.")
endif()

autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/image_transport_decompressor.cpp
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
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::image_preprocessor::ImageTransportDecompressor"
  EXECUTABLE image_transport_decompressor_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
