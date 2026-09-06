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

ament_auto_add_library(mppi_curvature_adaptive_steering_filter SHARED
  src/curvature_adaptive_steering_filter.cpp
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

  ament_add_gtest(test_curvature_adaptive_steering_filter
    test/test_curvature_adaptive_steering_filter.cpp
  )
  target_link_libraries(test_curvature_adaptive_steering_filter
    mppi_curvature_adaptive_steering_filter
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

if(CUDA_AVAIL AND TRT_AVAIL)
  find_package(MPPI-Generic REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  pluginlib_export_plugin_description_file(
    autoware_trajectory_modifier trajectory_modifier_plugins.xml)
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
  list(APPEND CUDA_NVCC_FLAGS "-lineinfo")
  list(APPEND CUDA_NVCC_FLAGS "--maxrregcount=64")

  # MPPI-Generic includes <cuda/barrier>, which #errors below sm_70.
  # Do not use CUDA_SELECT_NVCC_ARCH_FLAGS(Auto): container/CI configure (no
  # bashrc, virtual GPU, or FindCUDA fallback) can still emit a pre-Volta pass
  # such as compute_53, which then fails while compiling cuda/std/barrier.
  # Match Autoware perception TensorRT packages: explicit Turing+ fatbin.
  # Local override: -DMPPI_CUDA_ARCHITECTURES=86 (or a FindCUDA arch list).
  if(DEFINED MPPI_CUDA_ARCHITECTURES AND NOT MPPI_CUDA_ARCHITECTURES STREQUAL "")
    CUDA_SELECT_NVCC_ARCH_FLAGS(MPPI_CUDA_ARCH_FLAGS ${MPPI_CUDA_ARCHITECTURES})
    if(MPPI_CUDA_ARCH_FLAGS STREQUAL "")
      message(FATAL_ERROR
        "MPPI CUDA: MPPI_CUDA_ARCHITECTURES='${MPPI_CUDA_ARCHITECTURES}' "
        "produced no NVCC arch flags")
    endif()
    list(APPEND CUDA_NVCC_FLAGS ${MPPI_CUDA_ARCH_FLAGS})
    message(STATUS "MPPI CUDA architecture(s) (override): ${MPPI_CUDA_ARCH_FLAGS}")
  else()
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_75,code=sm_75")
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_86,code=sm_86")
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_87,code=sm_87")
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=sm_89")
    if(CUDA_VERSION VERSION_GREATER_EQUAL "12.8")
      if(CUDA_VERSION VERSION_LESS "13.0")
        list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_101,code=sm_101")
      else()
        list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_110,code=sm_110")
      endif()
      list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_120,code=sm_120")
      list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_120,code=compute_120")
    else()
      list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=compute_89")
    endif()
    message(STATUS "MPPI CUDA architectures: Autoware perception set (sm_75+)")
  endif()

  cuda_add_library(first_order_dubins_mppi SHARED
    src/first_order_dubins/distance_map_texture_visualizer.cu
    src/first_order_dubins/first_order_dubins_mppi_interface.cu
  )
  find_package(autoware_trajectory_modifier REQUIRED)
  ament_auto_add_library(mppi_temporal_mpt_nominal SHARED
    src/first_order_dubins/temporal_mpt_nominal.cpp
  )
  target_link_libraries(mppi_temporal_mpt_nominal
    mppi_trajectory_utils
    ${autoware_trajectory_modifier_LIBRARIES}
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
    OpenGL::GL
    glfw
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

  ament_auto_add_library(autoware_mppi_trajectory_modifier_plugin SHARED
    src/trajectory_mppi_optimizer.cpp
  )
  target_link_libraries(autoware_mppi_trajectory_modifier_plugin
    first_order_dubins_mppi
    mppi_curvature_adaptive_steering_filter
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
    mppi_curvature_adaptive_steering_filter
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

    find_package(rcutils REQUIRED)
    ament_add_gtest(test_performance_benchmarks
      test/test_performance_benchmarks.cpp
      TIMEOUT 1800
    )
    target_link_libraries(test_performance_benchmarks
      first_order_dubins_mppi
      rcutils::rcutils
      ${CUDA_LIBRARIES}
    )
    set_target_properties(test_performance_benchmarks PROPERTIES
      CXX_STANDARD 17
    )
    # Prevent other tests in this CTest invocation from contending for the benchmark GPU.
    set_tests_properties(test_performance_benchmarks PROPERTIES
      LABELS "gpu;performance"
      RUN_SERIAL TRUE
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

    # Keep the descriptive .cpp filename while compiling the texture-sampling test kernels with
    # NVCC. FindCUDA treats a non-.cu source as CUDA when this property is set.
    set_source_files_properties(test/test_distance_map.cpp PROPERTIES
      CUDA_SOURCE_PROPERTY_FORMAT OBJ
    )
    cuda_add_executable(test_distance_map
      test/test_distance_map.cpp
    )
    target_link_libraries(test_distance_map
      first_order_dubins_mppi
      ${GTEST_LIBRARIES}
      ${GTEST_MAIN_LIBRARIES}
    )
    target_include_directories(test_distance_map SYSTEM PRIVATE
      ${GTEST_INCLUDE_DIRS}
    )
    set_target_properties(test_distance_map PROPERTIES
      CXX_STANDARD 17
      CUDA_STANDARD 17
    )
    target_compile_options(test_distance_map PRIVATE
      $<$<COMPILE_LANG_AND_ID:CUDA,NVIDIA>:-Xcudafe;--diag_suppress=177>
    )
    ament_add_gtest_test(test_distance_map)
    set_tests_properties(test_distance_map PROPERTIES
      LABELS "gpu"
      TIMEOUT 180
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
if(CUDA_AVAIL AND TRT_AVAIL)
  ament_export_libraries(first_order_dubins_mppi)
  ament_export_dependencies(MPPI-Generic OpenGL glfw3)
endif()
ament_auto_package(INSTALL_TO_SHARE config)
