cmake_minimum_required(VERSION 3.14)
project(autoware_mppi_optimizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

generate_parameter_library(trajectory_mppi_optimizer_parameters
  param/trajectory_mppi_optimizer_parameters.yaml
)

ament_auto_add_library(mppi_trajectory_utils SHARED
  src/first_order_dubins/trajectory_utils.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_mppi_trajectory_utils
    test/test_trajectory_utils.cpp
  )
  target_link_libraries(test_mppi_trajectory_utils
    mppi_trajectory_utils
  )
endif()

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

# MPPI-Generic requires cuda::barrier (sm_70+). Skip the CUDA plugin on builders without a
# suitable GPU unless MPPI_CUDA_ARCH_LIST / CMAKE_CUDA_ARCHITECTURES is set for cross-compile.
set(MPPI_BUILD_CUDA OFF)
if(CUDA_AVAIL AND TRT_AVAIL)
  set(MPPI_CUDA_ARCH_LIST "" CACHE STRING
    "CUDA architectures for MPPI cross-compile (e.g. 87). Empty = auto-detect GPU via nvidia-smi.")
  set(MPPI_MIN_CUDA_ARCH 70 CACHE STRING
    "Minimum CUDA compute capability for MPPI (CUDA barriers require sm_70+).")

  # Do not inherit global CMAKE_CUDA_ARCHITECTURES from colcon (CI sets 87 for other
  # packages). MPPI only cross-compiles when MPPI_CUDA_ARCH_LIST is set explicitly.
  set(_mppi_cuda_arch_spec Auto)
  if(NOT MPPI_CUDA_ARCH_LIST STREQUAL "")
    set(_mppi_cuda_arch_spec ${MPPI_CUDA_ARCH_LIST})
  endif()

  set(_mppi_gpu_cc "")
  if(_mppi_cuda_arch_spec STREQUAL "Auto")
    execute_process(
      COMMAND nvidia-smi --query-gpu=compute_cap --format=csv,noheader,nounits
      OUTPUT_VARIABLE _mppi_gpu_cc
      ERROR_QUIET
      OUTPUT_STRIP_TRAILING_WHITESPACE)
  endif()

  set(_mppi_sm_targets "")
  if(_mppi_cuda_arch_spec STREQUAL "Auto")
    if(NOT _mppi_gpu_cc STREQUAL "")
      string(REPLACE "\n" ";" _mppi_gpu_cc_list "${_mppi_gpu_cc}")
      list(GET _mppi_gpu_cc_list 0 _mppi_gpu_cc_primary)
      string(REPLACE "." "" _mppi_gpu_sm "${_mppi_gpu_cc_primary}")
      list(APPEND _mppi_sm_targets ${_mppi_gpu_sm})
    endif()
  else()
    string(REPLACE ";" " " _mppi_arch_spec_normalized "${_mppi_cuda_arch_spec}")
    string(REPLACE " " ";" _mppi_arch_spec_normalized "${_mppi_arch_spec_normalized}")
    string(REPLACE "," ";" _mppi_arch_spec_normalized "${_mppi_arch_spec_normalized}")
    set(_mppi_sm_targets ${_mppi_arch_spec_normalized})
  endif()

  set(_mppi_skip_reason "")
  if(_mppi_sm_targets STREQUAL "")
    if(_mppi_cuda_arch_spec STREQUAL "Auto")
      set(_mppi_skip_reason
        "no GPU detected and MPPI_CUDA_ARCH_LIST not set")
    else()
      set(_mppi_skip_reason
        "failed to parse CUDA architecture spec '${_mppi_cuda_arch_spec}'")
    endif()
  else()
    foreach(_mppi_sm IN LISTS _mppi_sm_targets)
      if(_mppi_sm LESS MPPI_MIN_CUDA_ARCH)
        set(_mppi_skip_reason
          "target architecture sm_${_mppi_sm} < required sm_${MPPI_MIN_CUDA_ARCH}")
        break()
      endif()
    endforeach()
  endif()

  set(MPPI_CUDA_ARCH_FLAGS "")
  if(_mppi_skip_reason STREQUAL "")
    # CMake 3.22 FindCUDA/select_compute_arch.cmake does not know sm_87+ (Jetson Orin, Ada, etc.).
    set(_mppi_cmake322_known_sms
      20 21 30 32 35 37 50 52 53 60 61 62 70 72 75 80 86)
    set(_mppi_legacy_sm_targets "")
    foreach(_mppi_sm IN LISTS _mppi_sm_targets)
      if(_mppi_sm IN_LIST _mppi_cmake322_known_sms)
        list(APPEND _mppi_legacy_sm_targets ${_mppi_sm})
      else()
        list(APPEND MPPI_CUDA_ARCH_FLAGS "-gencode" "arch=compute_${_mppi_sm},code=sm_${_mppi_sm}")
      endif()
    endforeach()
    if(_mppi_legacy_sm_targets)
      CUDA_SELECT_NVCC_ARCH_FLAGS(_mppi_legacy_arch_flags ${_mppi_legacy_sm_targets})
      list(APPEND MPPI_CUDA_ARCH_FLAGS ${_mppi_legacy_arch_flags})
    endif()
    if(MPPI_CUDA_ARCH_FLAGS STREQUAL "")
      set(_mppi_skip_reason
        "failed to resolve NVCC gencode flags from '${_mppi_cuda_arch_spec}'")
    else()
      set(MPPI_BUILD_CUDA ON)
    endif()
  endif()

  if(NOT MPPI_BUILD_CUDA)
    message(WARNING
      "autoware_mppi_optimizer: skipping MPPI CUDA plugin build (${_mppi_skip_reason}). "
      "Install trajectory_utils/parameters only. To cross-compile, set "
      "MPPI_CUDA_ARCH_LIST (e.g. -DMPPI_CUDA_ARCH_LIST=87).")
  else()
    message(STATUS "autoware_mppi_optimizer: building MPPI CUDA for sm ${_mppi_sm_targets}")
    message(STATUS "MPPI CUDA architecture(s): ${MPPI_CUDA_ARCH_FLAGS}")
  endif()
endif()

if(MPPI_BUILD_CUDA)
  find_package(MPPI-Generic REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  pluginlib_export_plugin_description_file(
    autoware_trajectory_processor trajectory_processor_plugins.xml)
  add_library(mppi_extensions INTERFACE)
  add_library(MppiExtensions::extensions ALIAS mppi_extensions)
  target_include_directories(mppi_extensions BEFORE INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(mppi_extensions INTERFACE MPPI::MPPI)

  set(CUDA_PROPAGATE_HOST_FLAGS OFF)
  list(APPEND CUDA_NVCC_FLAGS "--expt-relaxed-constexpr -diag-suppress 20012")
  list(APPEND CUDA_NVCC_FLAGS "--extended-lambda")
  list(APPEND CUDA_NVCC_FLAGS "-Xcompiler=-std=c++17")
  # Parallelize NVCC device-code passes (CUDA 11.5+); 0 = use all host cores.
  list(APPEND CUDA_NVCC_FLAGS "--threads" "0")
  list(APPEND CUDA_NVCC_FLAGS ${MPPI_CUDA_ARCH_FLAGS})

  cuda_add_library(first_order_dubins_mppi SHARED
    src/first_order_dubins/first_order_dubins_mppi_interface.cu
  )
  find_package(autoware_trajectory_processor REQUIRED)
  ament_auto_add_library(mppi_temporal_mpt_nominal SHARED
    src/first_order_dubins/temporal_mpt_nominal.cpp
  )
  target_link_libraries(mppi_temporal_mpt_nominal
    mppi_trajectory_utils
    ${autoware_trajectory_processor_LIBRARIES}
    rclcpp::rclcpp
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    autoware_planning_msgs::autoware_planning_msgs__rosidl_generator_cpp
  )
  target_include_directories(first_order_dubins_mppi PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    SYSTEM ${CUDA_INCLUDE_DIRS}
  )
  target_link_libraries(first_order_dubins_mppi
    mppi_trajectory_utils
    mppi_temporal_mpt_nominal
    mppi_extensions
    rclcpp::rclcpp
    tf2_geometry_msgs::tf2_geometry_msgs
    autoware_perception_msgs::autoware_perception_msgs__rosidl_generator_cpp
    autoware_planning_msgs::autoware_planning_msgs__rosidl_generator_cpp
    autoware_vehicle_msgs::autoware_vehicle_msgs__rosidl_generator_cpp
    geometry_msgs::geometry_msgs__rosidl_generator_cpp
    nav_msgs::nav_msgs__rosidl_generator_cpp
    ${CUDA_cufft_LIBRARY}
  )
  set_target_properties(first_order_dubins_mppi PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 17
    CUDA_STANDARD 17
  )

  ament_auto_add_library(${PROJECT_NAME} SHARED
    src/first_order_dubins_mppi_cost_params_ros.cpp
    src/first_order_dubins_mppi_runtime_options_ros.cpp
    src/first_order_dubins_mppi_vehicle_params_ros.cpp
  )
  target_link_libraries(${PROJECT_NAME} first_order_dubins_mppi)

  ament_auto_add_library(autoware_mppi_trajectory_processor_plugin SHARED
    src/trajectory_mppi_optimizer.cpp
  )
  target_link_libraries(autoware_mppi_trajectory_processor_plugin
    first_order_dubins_mppi
    trajectory_mppi_optimizer_parameters
    ${PROJECT_NAME}
  )

  add_executable(mppi_offline_retune
    src/tools/mppi_offline_retune.cpp
  )
  add_executable(mppi_open_loop_line_sim
    src/tools/mppi_open_loop_line_sim.cpp
  )
  target_include_directories(mppi_offline_retune PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(mppi_offline_retune
    first_order_dubins_mppi
    rclcpp::rclcpp
    tf2_geometry_msgs::tf2_geometry_msgs
    autoware_perception_msgs::autoware_perception_msgs__rosidl_generator_cpp
    autoware_planning_msgs::autoware_planning_msgs__rosidl_generator_cpp
    autoware_vehicle_msgs::autoware_vehicle_msgs__rosidl_generator_cpp
    geometry_msgs::geometry_msgs__rosidl_generator_cpp
    nav_msgs::nav_msgs__rosidl_generator_cpp
  )
  set_target_properties(mppi_offline_retune PROPERTIES
    CXX_STANDARD 17
    POSITION_INDEPENDENT_CODE ON
  )

  target_include_directories(mppi_open_loop_line_sim PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(mppi_open_loop_line_sim
    first_order_dubins_mppi
    ${PROJECT_NAME}
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    tf2_geometry_msgs::tf2_geometry_msgs
    autoware_perception_msgs::autoware_perception_msgs__rosidl_generator_cpp
    autoware_planning_msgs::autoware_planning_msgs__rosidl_generator_cpp
    autoware_vehicle_msgs::autoware_vehicle_msgs__rosidl_generator_cpp
    geometry_msgs::geometry_msgs__rosidl_generator_cpp
    nav_msgs::nav_msgs__rosidl_generator_cpp
  )
  set_target_properties(mppi_open_loop_line_sim PROPERTIES
    CXX_STANDARD 17
    POSITION_INDEPENDENT_CODE ON
  )

  if(BUILD_TESTING)
    ament_find_gtest()

    ament_add_gtest(test_first_order_dubins_mppi_interface
      test/test_first_order_dubins_mppi_interface.cpp
    )
    target_link_libraries(test_first_order_dubins_mppi_interface
      first_order_dubins_mppi
      mppi_trajectory_utils
      ${CUDA_LIBRARIES}
    )
    set_target_properties(test_first_order_dubins_mppi_interface PROPERTIES
      CXX_STANDARD 17
      CUDA_STANDARD 17
    )
    target_compile_options(test_first_order_dubins_mppi_interface PRIVATE
      $<$<COMPILE_LANG_AND_ID:CUDA,NVIDIA>:-Xcudafe;--diag_suppress=177>
    )
    set_tests_properties(test_first_order_dubins_mppi_interface PROPERTIES
      LABELS "gpu"
      TIMEOUT 180
    )

    cuda_add_executable(test_mppi_trajectory_validator
      test/test_trajectory_validator.cu
    )
    target_link_libraries(test_mppi_trajectory_validator
      first_order_dubins_mppi
      ${GTEST_LIBRARIES}
      ${GTEST_MAIN_LIBRARIES}
    )
    target_include_directories(test_mppi_trajectory_validator SYSTEM PRIVATE
      ${GTEST_INCLUDE_DIRS}
    )
    set_target_properties(test_mppi_trajectory_validator PROPERTIES
      CXX_STANDARD 17
      CUDA_STANDARD 17
    )
    target_compile_options(test_mppi_trajectory_validator PRIVATE
      $<$<COMPILE_LANG_AND_ID:CUDA,NVIDIA>:-Xcudafe;--diag_suppress=177>
    )
    ament_add_gtest_test(test_mppi_trajectory_validator)
    set_tests_properties(test_mppi_trajectory_validator PROPERTIES
      LABELS "gpu"
      TIMEOUT 60
    )
  endif()

  install(TARGETS first_order_dubins_mppi mppi_offline_retune mppi_open_loop_line_sim
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  install(PROGRAMS
    scripts/mppi_offline_tuner.py
    scripts/mppi_debug_visualizer.py
    scripts/mppi_open_loop_line_sim_plot.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_export_include_directories(include)
if(MPPI_BUILD_CUDA)
  ament_export_libraries(first_order_dubins_mppi)
  ament_export_dependencies(MPPI-Generic)
endif()
ament_auto_package(INSTALL_TO_SHARE config)
