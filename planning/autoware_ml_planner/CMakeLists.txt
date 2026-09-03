cmake_minimum_required(VERSION 3.17)
project(autoware_ml_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)
find_package(xtensor REQUIRED)

option(ENABLE_ONNXRUNTIME "Enable ONNX Runtime inference when available" ON)
option(ENABLE_ACADOS "Enable acados trajectory optimization when available" ON)
option(ML_PLANNER_VERBOSE_DEPENDENCIES "Print optional dependency details" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(FindMLPlannerDependencies)
include(AddAcadosSolver)

ml_planner_find_dependencies()

set(ML_PLANNER_SOURCES
  src/ml_planner_core.cpp
  src/ml_planner_node.cpp
  src/inference/single_step_inference.cpp
  src/inference/utils.cpp
  src/postprocessing/postprocessing_utils.cpp
  src/postprocessing/road_border_avoidance.cpp
  src/preprocessing/input_builder.cpp
  src/preprocessing/items/agent.cpp
  src/preprocessing/items/ego_history.cpp
  src/preprocessing/items/ego_shape.cpp
  src/preprocessing/items/goal_pose.cpp
  src/preprocessing/items/initial_noise.cpp
  src/preprocessing/items/map.cpp
  src/preprocessing/items/traffic_signals.cpp
  src/preprocessing/items/turn_indicators.cpp
  src/preprocessing/preprocessing_utils.cpp
  src/utils/marker_utils.cpp
  src/utils/planning_factor_utils.cpp
  src/utils/utils.cpp
)

if(ML_PLANNER_HAS_ONNXRUNTIME)
  list(APPEND ML_PLANNER_SOURCES src/inference/onnxruntime_inference.cpp)
endif()

set(ML_PLANNER_HAS_ACADOS FALSE)
if(ENABLE_ACADOS)
  ml_planner_add_acados_solver(ML_PLANNER_ACADOS_TARGET)
  if(ML_PLANNER_ACADOS_TARGET)
    set(ML_PLANNER_HAS_ACADOS TRUE)
    list(APPEND ML_PLANNER_SOURCES src/optimization/trajectory_optimizer.cpp)
  endif()
endif()

ament_auto_add_library(autoware_ml_planner_component SHARED
  ${ML_PLANNER_SOURCES}
)

target_include_directories(autoware_ml_planner_component PUBLIC
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(autoware_ml_planner_component
  CUDA::cudart
  CUDA::cublas
  MLPlanner::nvinfer
  MLPlanner::nvonnxparser
  xtensor
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(autoware_ml_planner_component PRIVATE
    -Wno-deprecated-declarations
    -Wno-narrowing
  )
endif()

if(ML_PLANNER_HAS_ONNXRUNTIME)
  target_link_libraries(autoware_ml_planner_component
    MLPlanner::onnxruntime
  )
  target_compile_definitions(autoware_ml_planner_component PUBLIC
    AUTOWARE_ML_PLANNER_USE_ONNXRUNTIME
  )
endif()

if(ML_PLANNER_HAS_ACADOS)
  target_link_libraries(autoware_ml_planner_component
    ${ML_PLANNER_ACADOS_TARGET}
  )
  target_compile_definitions(autoware_ml_planner_component PUBLIC
    AUTOWARE_ML_PLANNER_USE_ACADOS
  )
endif()

# Make the acados shared libraries discoverable after sourcing the workspace setup file.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/acados_library_path.dsv.in")

autoware_agnocast_wrapper_register_node(autoware_ml_planner_component
  PLUGIN "autoware::ml_planner::MLPlanner"
  EXECUTABLE autoware_ml_planner_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
)

if(BUILD_TESTING)
  set(ML_PLANNER_TEST_SOURCES
    test/lanelet_edge_case_test.cpp
    test/lanelet_integration_test.cpp
    test/lanelet_test.cpp
    test/marker_utils_test.cpp
    test/planning_factor_utils_test.cpp
    test/postprocessing_utils_test.cpp
    test/postprocessing_utils_edge_case_test.cpp
    test/road_border_avoidance_test.cpp
    test/utils_test.cpp
  )

  # Keep the optimizer test conditional because its public API is only available with acados.
  if(ML_PLANNER_HAS_ACADOS)
    list(APPEND ML_PLANNER_TEST_SOURCES test/trajectory_optimizer_test.cpp)
  endif()

  ament_add_ros_isolated_gtest(test_ml_planner
    ${ML_PLANNER_TEST_SOURCES}
  )
  target_link_libraries(test_ml_planner autoware_ml_planner_component)
  ament_target_dependencies(test_ml_planner
    autoware_lanelet2_extension
    autoware_map_msgs
    autoware_perception_msgs
    autoware_test_utils
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
    test_map
)
