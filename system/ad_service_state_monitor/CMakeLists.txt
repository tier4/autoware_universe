cmake_minimum_required(VERSION 3.5)
project(ad_service_state_monitor)

set(CMAKE_BUILD_TYPE "Debug")

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(agnocastlib REQUIRED)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Target
## Set source files
set(AD_SERVICE_STATE_MONITOR_SRC
  src/ad_service_state_monitor_node/ad_service_state_monitor_node.cpp
  src/ad_service_state_monitor_node/state_machine.cpp
  src/ad_service_state_monitor_node/diagnostics.cpp
)

## Add executables
ament_auto_add_executable(ad_service_state_monitor
  src/ad_service_state_monitor_node/main.cpp
  ${AD_SERVICE_STATE_MONITOR_SRC}
)

ament_target_dependencies(ad_service_state_monitor agnocastlib)
target_include_directories(ad_service_state_monitor PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

target_link_libraries(ad_service_state_monitor glog)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
