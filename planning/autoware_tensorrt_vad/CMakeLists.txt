cmake_minimum_required(VERSION 3.14)

project(autoware_tensorrt_vad)

find_package(autoware_cmake REQUIRED)

autoware_package(
    SKIP_LINT
)

# Set C++17 standard to match Autoware Universe
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Find additional dependencies for components
find_package(CUDA REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# TensorRT settings
option(TRT_ROOT "$ENV{TRT_ROOT}")
if(TRT_ROOT)
    set(TENSORRT_INCLUDE_DIRS ${TRT_ROOT}/include/)
    set(TENSORRT_LIBRARY_DIRS ${TRT_ROOT}/lib/)
else()
    # Default TensorRT paths
    set(TENSORRT_INCLUDE_DIRS /usr/include/x86_64-linux-gnu)
    set(TENSORRT_LIBRARY_DIRS /usr/lib/x86_64-linux-gnu)
endif()

# Enable CUDA language
enable_language(CUDA)

# CUDA settings
set(CUDA_INCLUDE_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/include)
set(CUDA_LIBRARY_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/lib64)

# Set CUDA architecture
set(CMAKE_CUDA_ARCHITECTURES 75 80 86)
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

# Separate compile options for C++ and CUDA
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

# Explicitly disable Werror for CUDA
set(CMAKE_CUDA_FLAGS "")

# Include directory settings
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ${CUDA_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# Link directory settings
link_directories(
    ${CUDA_LIBRARY_DIRS}
    ${TENSORRT_LIBRARY_DIRS}
)

# Compile CUDA kernel separately
set_source_files_properties(
    lib/networks/preprocess/multi_camera_preprocess_kernel.cu
    PROPERTIES 
    LANGUAGE CUDA
    COMPILE_FLAGS "-w"
)

# Create separate CUDA library to isolate compilation issues
add_library(${PROJECT_NAME}_cuda_lib STATIC
    lib/networks/preprocess/multi_camera_preprocess_kernel.cu
)

set_target_properties(${PROJECT_NAME}_cuda_lib PROPERTIES
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    POSITION_INDEPENDENT_CODE ON
    CUDA_RUNTIME_LIBRARY Shared
    CUDA_STANDARD 17
    CUDA_STANDARD_REQUIRED ON
)

target_include_directories(${PROJECT_NAME}_cuda_lib PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ${CUDA_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# Link OpenCV to CUDA library if needed
target_link_libraries(${PROJECT_NAME}_cuda_lib PRIVATE
    ${OpenCV_LIBS}
)

# Override all inherited compile options for CUDA library
set_target_properties(${PROJECT_NAME}_cuda_lib PROPERTIES
    COMPILE_OPTIONS ""
    COMPILE_FLAGS ""
)

# Set minimal CUDA-specific compile options
target_compile_options(${PROJECT_NAME}_cuda_lib PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:-w>
)

# Add main library (excluding CUDA files)
ament_auto_add_library(${PROJECT_NAME}_lib SHARED
    lib/vad_interface.cpp
    lib/vad_model.cpp
    lib/vad_config.cpp
    lib/synchronization_strategy.cpp
    lib/coordinate_transformer.cpp
    lib/converter.cpp
    lib/input_converter/image_converter.cpp
    lib/input_converter/transform_matrix_converter.cpp
    lib/input_converter/can_bus_converter.cpp
    lib/input_converter/bev_shift_converter.cpp
    lib/output_converter/trajectory_converter.cpp
    lib/output_converter/map_converter.cpp
    lib/output_converter/objects_converter.cpp
    lib/networks/net.cpp
    lib/networks/backbone.cpp
    lib/networks/head.cpp
    lib/networks/tensor.cpp
    lib/networks/preprocess/multi_camera_preprocess.cpp
)

# Set properties for the main library
set_target_properties(${PROJECT_NAME}_lib PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# Library link settings
target_link_libraries(${PROJECT_NAME}_lib
    ${PROJECT_NAME}_cuda_lib
    nvinfer
    ${CUDA_LIBRARIES}
    cuda
    cudart
    ${OpenCV_LIBS}
)

# Add component
ament_auto_add_library(${PROJECT_NAME}_component SHARED
    src/vad_node.cpp
)

target_link_libraries(${PROJECT_NAME}_component
    ${PROJECT_NAME}_lib
)

rclcpp_components_register_node(${PROJECT_NAME}_component
    PLUGIN "autoware::tensorrt_vad::VadNode"
    EXECUTABLE vad_node
)

ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE
        launch
        config
)
