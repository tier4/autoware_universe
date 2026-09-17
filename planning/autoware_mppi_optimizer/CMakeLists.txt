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

if(CUDA_AVAIL AND TRT_AVAIL)
  find_package(MPPI-Generic REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  pluginlib_export_plugin_description_file(
    autoware_trajectory_processor trajectory_processor_plugins.xml)
  add_library(mppi_extensions INTERFACE)
  add_library(MppiExtensions::extensions ALIAS mppi_extensions)
  target_include_directories(mppi_extensions BEFORE INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(mppi_extensions INTERFACE MPPI::MPPI)

  # acados reaches this package transitively, through autoware_trajectory_processor.
  # libacados.so lists libqpOASES_e.so / libhpipm.so / libblasfeo.so.0 as DT_NEEDED
  # but carries no usable search path for them, and we link the trajectory_processor
  # libraries directly instead of through ament_target_dependencies -- so the
  # "-L<acados>/lib" that autoware_trajectory_processor publishes via
  # ament_export_link_flags never reaches our link line. ld then cannot locate those
  # transitive DSOs and every acados symbol comes back undefined.
  # Same workaround as autoware_trajectory_processor/CMakeLists.txt: resolve the
  # acados lib dir, link its runtime libs directly so they land as DT_NEEDED on our
  # own binaries (surviving pluginlib dlopen without LD_LIBRARY_PATH), and record
  # the directory in the build/install RPATH.
  find_package(acados QUIET)
  set(_mppi_acados_lib_dir "")
  if(TARGET acados)
    get_target_property(_mppi_acados_location acados IMPORTED_LOCATION)
    if(NOT _mppi_acados_location)
      get_target_property(_mppi_acados_location acados IMPORTED_LOCATION_RELEASE)
    endif()
    if(_mppi_acados_location)
      get_filename_component(_mppi_acados_lib_dir "${_mppi_acados_location}" DIRECTORY)
    endif()
  endif()
  if(NOT _mppi_acados_lib_dir)
    # Fallback when acados is not exposed as an imported target (e.g. in-tree)
    set(_mppi_acados_lib_dir "$ENV{ACADOS_SOURCE_DIR}/lib")
  endif()
  if(NOT EXISTS "${_mppi_acados_lib_dir}")
    set(_mppi_acados_lib_dir "")
    message(WARNING
      "MPPI acados: could not locate the acados lib directory. Linking against "
      "acados' transitive dependencies may fail; set ACADOS_SOURCE_DIR.")
  endif()

  set(_mppi_acados_runtime_libs "")
  if(_mppi_acados_lib_dir)
    foreach(_mppi_acados_dep qpOASES_e hpipm blasfeo)
      find_library(_mppi_acados_dep_path NAMES ${_mppi_acados_dep}
        PATHS "${_mppi_acados_lib_dir}" NO_DEFAULT_PATH)
      if(NOT _mppi_acados_dep_path)
        find_library(_mppi_acados_dep_path NAMES ${_mppi_acados_dep})
      endif()
      if(_mppi_acados_dep_path)
        list(APPEND _mppi_acados_runtime_libs "${_mppi_acados_dep_path}")
      endif()
      unset(_mppi_acados_dep_path CACHE)
    endforeach()
  endif()

  # Attach the resolved acados runtime to one target. -rpath-link lets ld resolve
  # the transitive DSOs while linking; --no-as-needed keeps them as DT_NEEDED even
  # though this package references none of their symbols directly. The as-needed
  # pair is passed through target_link_libraries rather than target_link_options so
  # that it actually brackets the libraries on the generated link line.
  function(mppi_link_acados_runtime target)
    if(NOT _mppi_acados_lib_dir)
      return()
    endif()
    if(UNIX AND NOT APPLE)
      target_link_options(${target} PRIVATE
        "LINKER:-rpath-link,${_mppi_acados_lib_dir}")
    endif()
    if(_mppi_acados_runtime_libs)
      if(UNIX AND NOT APPLE)
        target_link_libraries(${target}
          "-Wl,--no-as-needed" ${_mppi_acados_runtime_libs} "-Wl,--as-needed")
      else()
        target_link_libraries(${target} ${_mppi_acados_runtime_libs})
      endif()
    endif()
    set_property(TARGET ${target} APPEND PROPERTY
      BUILD_RPATH "${_mppi_acados_lib_dir}")
    set_property(TARGET ${target} APPEND PROPERTY
      INSTALL_RPATH "${_mppi_acados_lib_dir}")
  endfunction()

  set(CUDA_PROPAGATE_HOST_FLAGS OFF)
  list(APPEND CUDA_NVCC_FLAGS "--expt-relaxed-constexpr -diag-suppress 20012")
  list(APPEND CUDA_NVCC_FLAGS "--extended-lambda")
  list(APPEND CUDA_NVCC_FLAGS "-Xcompiler=-std=c++17")
  # Parallelize NVCC device-code passes (CUDA 11.5+); 0 = use all host cores.
  list(APPEND CUDA_NVCC_FLAGS "--threads" "0")

  # CUDA architectures to build device code for.
  #   Default : the portable gencode list shared with the perception/sensing CUDA
  #             packages. Needs no GPU at configure time, so it also works on
  #             headless build machines and in containers started without --gpus.
  #   Auto    : detect the GPU(s) on this machine and build only for those. Much
  #             faster -- MPPI-Generic is template-heavy and every extra SM is a
  #             full device-code pass -- but only valid when the build host has
  #             the target GPU.
  #   <list>  : an explicit capability list, e.g. -DMPPI_CUDA_ARCH=8.6, passed
  #             through to CUDA_SELECT_NVCC_ARCH_FLAGS.
  set(MPPI_CUDA_ARCH "Default" CACHE STRING
    "CUDA architectures for MPPI: Default, Auto, or an explicit list (e.g. 8.6)")

  if(MPPI_CUDA_ARCH STREQUAL "Default")
    set(MPPI_CUDA_ARCH_FLAGS
      "-gencode arch=compute_75,code=sm_75"
      "-gencode arch=compute_86,code=sm_86"
      "-gencode arch=compute_87,code=sm_87"
      "-gencode arch=compute_89,code=sm_89")
    if(CUDA_VERSION VERSION_GREATER_EQUAL "12.8")
      if(CUDA_VERSION VERSION_LESS "13.0")
        list(APPEND MPPI_CUDA_ARCH_FLAGS "-gencode arch=compute_101,code=sm_101")
      else()  # CUDA 13.0 renamed SM101 to SM110
        list(APPEND MPPI_CUDA_ARCH_FLAGS "-gencode arch=compute_110,code=sm_110")
      endif()
      list(APPEND MPPI_CUDA_ARCH_FLAGS "-gencode arch=compute_120,code=sm_120")
      list(APPEND MPPI_CUDA_ARCH_FLAGS "-gencode arch=compute_120,code=compute_120")
    else()
      list(APPEND MPPI_CUDA_ARCH_FLAGS "-gencode arch=compute_89,code=compute_89")
    endif()
  else()
    cuda_select_nvcc_arch_flags(MPPI_CUDA_ARCH_FLAGS ${MPPI_CUDA_ARCH})
    if(MPPI_CUDA_ARCH_FLAGS STREQUAL "")
      message(FATAL_ERROR
        "MPPI CUDA: MPPI_CUDA_ARCH=${MPPI_CUDA_ARCH} selected no architecture. "
        "With Auto, is the NVIDIA driver available at configure time? "
        "Use -DMPPI_CUDA_ARCH=Default to build the portable gencode list.")
    endif()
  endif()

  # MPPI-Generic includes <cuda/barrier>, and libcu++ hard-#errors below sm_70.
  # CUDA_SELECT_NVCC_ARCH_FLAGS(Auto) silently falls back to *every* known
  # architecture when no GPU is visible, which drags in sm_53 and buries the real
  # cause under a wall of libcu++ template errors -- so reject it here instead.
  foreach(mppi_arch_flag IN LISTS MPPI_CUDA_ARCH_FLAGS)
    string(REGEX MATCHALL "(compute|sm)_[0-9]+" mppi_arch_names "${mppi_arch_flag}")
    foreach(mppi_arch_name IN LISTS mppi_arch_names)
      string(REGEX REPLACE "^(compute|sm)_" "" mppi_arch_sm "${mppi_arch_name}")
      if(mppi_arch_sm LESS 70)
        message(FATAL_ERROR
          "MPPI CUDA: sm_${mppi_arch_sm} is below the sm_70 that MPPI-Generic "
          "requires via <cuda/barrier>. Selected by MPPI_CUDA_ARCH="
          "${MPPI_CUDA_ARCH}: ${MPPI_CUDA_ARCH_FLAGS}")
      endif()
    endforeach()
  endforeach()

  list(APPEND CUDA_NVCC_FLAGS ${MPPI_CUDA_ARCH_FLAGS})
  message(STATUS "MPPI CUDA architecture(s): ${MPPI_CUDA_ARCH_FLAGS}")

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
  # Entry point for acados here: ${autoware_trajectory_processor_LIBRARIES} above.
  mppi_link_acados_runtime(mppi_temporal_mpt_nominal)
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
  mppi_link_acados_runtime(first_order_dubins_mppi)
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
  mppi_link_acados_runtime(${PROJECT_NAME})

  ament_auto_add_library(autoware_mppi_trajectory_processor_plugin SHARED
    src/trajectory_mppi_optimizer.cpp
  )
  target_link_libraries(autoware_mppi_trajectory_processor_plugin
    first_order_dubins_mppi
    trajectory_mppi_optimizer_parameters
    ${PROJECT_NAME}
  )
  mppi_link_acados_runtime(autoware_mppi_trajectory_processor_plugin)

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
  mppi_link_acados_runtime(mppi_offline_retune)
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
    mppi_link_acados_runtime(test_first_order_dubins_mppi_interface)
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
    mppi_link_acados_runtime(test_mppi_trajectory_validator)
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
if(CUDA_AVAIL AND TRT_AVAIL)
  ament_export_libraries(first_order_dubins_mppi)
  ament_export_dependencies(MPPI-Generic)
endif()
ament_auto_package(INSTALL_TO_SHARE config)
