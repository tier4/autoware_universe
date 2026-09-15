cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_optimizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(acados REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_cmake_export_dependencies REQUIRED)
find_package(ament_cmake_export_link_flags REQUIRED)
ament_export_dependencies(acados)

get_target_property(_acados_location acados IMPORTED_LOCATION)
if(NOT _acados_location)
  get_target_property(_acados_location acados IMPORTED_LOCATION_RELEASE)
endif()
if(_acados_location)
  get_filename_component(_acados_lib_dir "${_acados_location}" DIRECTORY)
  ament_export_link_flags("-L${_acados_lib_dir}" "-lacados")
else()
  set(_acados_lib_dir "$ENV{ACADOS_SOURCE_DIR}/lib")
  if(_acados_lib_dir AND EXISTS "${_acados_lib_dir}")
    ament_export_link_flags("-L${_acados_lib_dir}" "-lacados")
  endif()
endif()

add_subdirectory(src/acados_time_sequence_raw)
pluginlib_export_plugin_description_file(autoware_trajectory_optimizer plugins.xml)

ament_auto_add_library(autoware_trajectory_optimizer_component SHARED
  src/trajectory_optimizer.cpp
  src/utils.cpp
)

rclcpp_components_register_node(autoware_trajectory_optimizer_component
  PLUGIN "autoware::trajectory_optimizer::TrajectoryOptimizer"
  EXECUTABLE autoware_trajectory_optimizer_node
)

ament_auto_add_library(autoware_trajectory_optimizer_plugins SHARED
  src/trajectory_optimizer_plugins/trajectory_eb_smoother_optimizer.cpp
  src/trajectory_optimizer_plugins/trajectory_extender.cpp
  src/trajectory_optimizer_plugins/trajectory_kinematic_feasibility_enforcer.cpp
  src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp
  src/trajectory_optimizer_plugins/trajectory_point_fixer.cpp
  src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp
  src/trajectory_optimizer_plugins/trajectory_spline_smoother.cpp
  src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp
  src/trajectory_optimizer_plugins/trajectory_time_sequence_raw_optimizer.cpp
  src/trajectory_optimizer_plugins/plugin_utils/continuous_jerk_smoother.cpp
  src/trajectory_optimizer_plugins/plugin_utils/trajectory_extender_utils.cpp
  src/trajectory_optimizer_plugins/plugin_utils/trajectory_mpt_optimizer_utils.cpp
  src/trajectory_optimizer_plugins/plugin_utils/trajectory_point_fixer_utils.cpp
  src/trajectory_optimizer_plugins/plugin_utils/trajectory_spline_smoother_utils.cpp
  src/trajectory_optimizer_plugins/plugin_utils/trajectory_velocity_optimizer_utils.cpp
  src/trajectory_optimizer_plugins/plugin_utils/time_sequence_trajectory_optimizer.cpp
  src/trajectory_optimizer_plugins/plugin_utils/road_border_avoidance.cpp
)

target_link_libraries(autoware_trajectory_optimizer_plugins
  acados_interface_time_seq
  tf2_geometry_msgs::tf2_geometry_msgs
)

# libqpOASES_e is normally only a transitive dependency of libacados. Link it
# directly so pluginlib/dlopen can resolve it without LD_LIBRARY_PATH.
if(TARGET qpOASES_e::qpOASES_e)
  set(_traj_opt_qpoases_lib qpOASES_e::qpOASES_e)
elseif(TARGET qpOASES_e)
  set(_traj_opt_qpoases_lib qpOASES_e)
else()
  find_library(_traj_opt_qpoases_lib_path NAMES qpOASES_e
    PATHS
      "${_acados_lib_dir}"
      "$ENV{ACADOS_SOURCE_DIR}/lib"
      /opt/acados/lib
    NO_DEFAULT_PATH)
  if(NOT _traj_opt_qpoases_lib_path)
    find_library(_traj_opt_qpoases_lib_path NAMES qpOASES_e)
  endif()
  if(_traj_opt_qpoases_lib_path)
    set(_traj_opt_qpoases_lib "${_traj_opt_qpoases_lib_path}")
  endif()
endif()
if(_traj_opt_qpoases_lib)
  if(UNIX AND NOT APPLE)
    target_link_options(autoware_trajectory_optimizer_plugins PRIVATE "LINKER:--no-as-needed")
  endif()
  target_link_libraries(autoware_trajectory_optimizer_plugins ${_traj_opt_qpoases_lib})
  if(UNIX AND NOT APPLE)
    target_link_options(autoware_trajectory_optimizer_plugins PRIVATE "LINKER:--as-needed")
  endif()
endif()
unset(_traj_opt_qpoases_lib_path CACHE)

if(_acados_lib_dir AND EXISTS "${_acados_lib_dir}")
  foreach(_target acados_interface_time_seq autoware_trajectory_optimizer_plugins)
    set_property(TARGET ${_target} APPEND PROPERTY BUILD_RPATH "${_acados_lib_dir}")
    set_property(TARGET ${_target} APPEND PROPERTY INSTALL_RPATH "${_acados_lib_dir}")
  endforeach()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE TEST_SOURCES tests/*.cpp)
  ament_add_ros_isolated_gtest(test_autoware_trajectory_optimizer
    ${TEST_SOURCES}
  )
  target_link_libraries(test_autoware_trajectory_optimizer
    autoware_trajectory_optimizer_component
    autoware_trajectory_optimizer_plugins
    acados_interface_time_seq
  )
endif()

install(TARGETS acados_interface_time_seq
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(acados_interface_time_seq)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
