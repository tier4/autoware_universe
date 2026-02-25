cmake_minimum_required(VERSION 3.5)
project(autoware_creep_guidance_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(autoware_creep_guidance_interface SHARED
  src/creep_guidance_interface.cpp
)

ament_auto_package()
