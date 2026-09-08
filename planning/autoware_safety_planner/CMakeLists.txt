cmake_minimum_required(VERSION 3.14)
project(autoware_safety_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

# The generated header lands in include/${PROJECT_NAME}/safety_planner_parameters.hpp
generate_parameter_library(safety_planner_parameters
  param/safety_planner_parameter_struct.yaml
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Helpers around the per-cycle input (PlannerContext) and the projected (s, l) views, shared by the
# node, the rough planner and the trajectory optimizer plugins. They live here rather than in the
# node library because the plugin .so files call them too.
# STATIC: as a SHARED library, linking the transitive .so files alone takes tens of minutes
ament_auto_add_library(${PROJECT_NAME}_common STATIC
  src/context.cpp
  src/utils/sl_view_utils.cpp
  src/utils/trajectory_conversion.cpp
)
target_link_libraries(${PROJECT_NAME}_common
  safety_planner_parameters
)
target_compile_options(${PROJECT_NAME}_common PRIVATE -Wno-deprecated-declarations)

ament_auto_add_library(${PROJECT_NAME}_constraint_plugins SHARED
  src/constraint_generator/lane_following_drivable_area.cpp
  src/constraint_generator/obstacle_stop.cpp
  src/constraint_generator/simple_drivable_area.cpp
  src/constraint_generator/vehicle_kinematics.cpp
)
target_link_libraries(${PROJECT_NAME}_constraint_plugins
  safety_planner_parameters
  ${PROJECT_NAME}_common
)
target_compile_options(${PROJECT_NAME}_constraint_plugins PRIVATE -Wno-deprecated-declarations)


# ---------------------------------------------------------------------------------------------
# acados code generation (Python -> C): the OCP that NlpTrajectoryOptimizer drives. Only the
# structure (the number of stages, the row layout, the slacks and the type of the cost) is fixed
# here; every number is injected at run time. Nothing is written into the source tree.
# ---------------------------------------------------------------------------------------------
set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}")
if(NOT ACADOS_SOURCE_DIR)
  set(ACADOS_SOURCE_DIR "/opt/acados")
endif()
find_package(acados REQUIRED)

set(NLP_MODEL_NAME rbp_nlp_time)
set(NLP_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/c_generated_code_nlp_time")
set(NLP_GENERATED_FILES
  ${NLP_GEN_DIR}/acados_solver_${NLP_MODEL_NAME}.h
  ${NLP_GEN_DIR}/acados_solver_${NLP_MODEL_NAME}.c
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_model/${NLP_MODEL_NAME}_model.h
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_model/${NLP_MODEL_NAME}_expl_ode_fun.c
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_model/${NLP_MODEL_NAME}_expl_vde_adj.c
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_model/${NLP_MODEL_NAME}_expl_vde_forw.c
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_constraints/${NLP_MODEL_NAME}_constraints.h
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_constraints/${NLP_MODEL_NAME}_constr_h_fun.c
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_constraints/${NLP_MODEL_NAME}_constr_h_fun_jac_uxt_zt.c
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_constraints/${NLP_MODEL_NAME}_constr_h_e_fun.c
  ${NLP_GEN_DIR}/${NLP_MODEL_NAME}_constraints/${NLP_MODEL_NAME}_constr_h_e_fun_jac_uxt_zt.c
)
set_source_files_properties(${NLP_GENERATED_FILES} PROPERTIES GENERATED TRUE)

add_custom_command(
  OUTPUT ${NLP_GENERATED_FILES}
  COMMAND ${CMAKE_COMMAND} -E env
    "ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}"
    "PYTHONPATH=${ACADOS_SOURCE_DIR}/interfaces/acados_template:${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}"
    ${ACADOS_SOURCE_DIR}/.venv/bin/python3
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/nlp_time_ocp.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/nlp_time_ocp.py
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/time_bicycle_model.py
  COMMENT "Generating the acados solver for NlpTrajectoryOptimizer"
)
add_custom_target(generate_acados_code_nlp_time DEPENDS ${NLP_GENERATED_FILES})

# The trajectory planner plugins; compiling the constraints and driving the rough planner and the
# optimizer are implementation details of this .so
ament_auto_add_library(${PROJECT_NAME}_trajectory_planner_plugins SHARED
  src/trajectory_planner/frenet_sampler/frenet_sampler.cpp
  src/trajectory_planner/nlp_planner/constraints_compiler.cpp
  src/trajectory_planner/nlp_planner/rough_planner.cpp
  src/trajectory_planner/nlp_planner/rough_optimizer_trajectory_planner.cpp
  src/trajectory_planner/nlp_planner/bezier.cpp
  src/trajectory_planner/nlp_planner/nlp_trajectory_optimizer.cpp
  src/trajectory_planner/nlp_planner/ssc_corridor.cpp
  src/trajectory_planner/nlp_planner/ssc_qp_trajectory_optimizer.cpp
  ${NLP_GENERATED_FILES}
)
add_dependencies(${PROJECT_NAME}_trajectory_planner_plugins generate_acados_code_nlp_time)
target_link_libraries(${PROJECT_NAME}_trajectory_planner_plugins
  safety_planner_parameters
  ${PROJECT_NAME}_common
  acados
)
target_include_directories(${PROJECT_NAME}_trajectory_planner_plugins PRIVATE
  # The generated headers are an implementation detail of the plugin, hence PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${ACADOS_SOURCE_DIR}/include
  ${ACADOS_SOURCE_DIR}/include/acados
  ${ACADOS_SOURCE_DIR}/include/acados_c
  ${ACADOS_SOURCE_DIR}/include/blasfeo/include
  ${ACADOS_SOURCE_DIR}/include/hpipm/include
)
target_compile_options(${PROJECT_NAME}_trajectory_planner_plugins PRIVATE -Wno-deprecated-declarations)

# The warnings of the generated C are not ours to fix
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  set_source_files_properties(${NLP_GENERATED_FILES} PROPERTIES
    COMPILE_OPTIONS "-Wno-unused-variable;-Wno-unused-parameter;-Wno-unused-function")
endif()

# The transitive dependencies of libacados (blasfeo, hpipm) exist only in the acados install tree,
# so an rpath is baked in and the caller does not have to set LD_LIBRARY_PATH
get_target_property(_acados_location acados IMPORTED_LOCATION)
if(NOT _acados_location)
  get_target_property(_acados_location acados IMPORTED_LOCATION_RELEASE)
endif()
if(_acados_location)
  get_filename_component(_acados_lib_dir "${_acados_location}" DIRECTORY)
else()
  set(_acados_lib_dir "${ACADOS_SOURCE_DIR}/lib")
endif()
set_target_properties(${PROJECT_NAME}_trajectory_planner_plugins PROPERTIES
  BUILD_RPATH "${_acados_lib_dir}"
  INSTALL_RPATH "${_acados_lib_dir}")

# libacados.so carries no rpath of its own, so its NEEDED entries (hpipm, blasfeo, qpOASES_e) are
# picked up from the default ldconfig path (/usr/local/lib). A libhpipm.so from another build there
# disagrees with libacados about the struct layout and segfaults while the solver is generated.
# The libraries of the acados tree are therefore added to the DT_NEEDED of the plugin by absolute
# path and pinned through DT_RPATH (--disable-new-dtags, which is searched before
# LD_LIBRARY_PATH), so the hpipm and blasfeo of that same tree are always loaded before
# libacados.
foreach(_acados_dep hpipm blasfeo qpOASES_e)
  if(EXISTS "${_acados_lib_dir}/lib${_acados_dep}.so")
    target_link_libraries(${PROJECT_NAME}_trajectory_planner_plugins
      "${_acados_lib_dir}/lib${_acados_dep}.so")
  endif()
endforeach()
target_link_options(${PROJECT_NAME}_trajectory_planner_plugins PRIVATE
  "LINKER:--disable-new-dtags")

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/safety_planner.cpp
  src/safety_planner_node.cpp
  src/utils/reference_path_smoother.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
  safety_planner_parameters
  ${PROJECT_NAME}_common
)
# The header generated by generate_parameter_library uses a deprecated alias of parameter_traits
target_compile_options(${PROJECT_NAME}_lib PRIVATE -Wno-deprecated-declarations)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "autoware::safety_planner::SafetyPlannerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
  find_package(pybind11_vendor REQUIRED)
  find_package(pybind11 REQUIRED)

  # Closed-loop simulation (test/closed_loop) that drives SafetyPlanner::plan without a node.
  # Numeric results (CSV) are always written under TEST_RESULTS_DIR (build tree, not tracked by git);
  # figures are written there too when EXPORT_TEST_PLOT_FIGURE is ON
  # (colcon build --cmake-args -DEXPORT_TEST_PLOT_FIGURE=ON).
  option(EXPORT_TEST_PLOT_FIGURE "Enable plotting in tests" OFF)

  file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    ${TEST_SOURCES}
    TIMEOUT 300
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}_lib
    ${PROJECT_NAME}_common
    safety_planner_parameters
    Python3::Python
    pybind11::embed
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  ament_target_dependencies(test_${PROJECT_NAME} autoware_pyplot autoware_test_utils)
  target_compile_options(test_${PROJECT_NAME} PRIVATE -Wno-deprecated-declarations)
  target_compile_definitions(test_${PROJECT_NAME} PRIVATE
    TEST_RESULTS_DIR="${CMAKE_CURRENT_BINARY_DIR}/test_results"
  )
  if(EXPORT_TEST_PLOT_FIGURE)
    message(STATUS "EXPORT_TEST_PLOT_FIGURE is ON")
    target_compile_definitions(test_${PROJECT_NAME} PRIVATE EXPORT_TEST_PLOT_FIGURE)
  endif()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
  test_data
)
