cmake_minimum_required(VERSION 3.5)
project(autoware_crosswalk_traffic_light_estimator)

find_package(autoware_cmake REQUIRED)
find_package(autoware_agnocast_wrapper REQUIRED)
autoware_package()

###########
## Build ##
###########

include_directories()

ament_auto_add_library(autoware_crosswalk_traffic_light_estimator SHARED
  src/node.cpp
)

target_include_directories(autoware_crosswalk_traffic_light_estimator PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
  ${autoware_agnocast_wrapper_INCLUDE_DIRS}
)

ament_target_dependencies(autoware_crosswalk_traffic_light_estimator
  autoware_agnocast_wrapper
  autoware_utils_debug
)

autoware_agnocast_wrapper_setup(autoware_crosswalk_traffic_light_estimator)

autoware_agnocast_wrapper_register_node(autoware_crosswalk_traffic_light_estimator
  PLUGIN "autoware::crosswalk_traffic_light_estimator::CrosswalkTrafficLightEstimatorNode"
  EXECUTABLE crosswalk_traffic_light_estimator_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
        INSTALL_TO_SHARE
        launch
        config
)
