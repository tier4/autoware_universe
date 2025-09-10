cmake_minimum_required(VERSION 3.14)
project(autoware_lanelet2_extension_python)

find_package(autoware_cmake REQUIRED)
find_package(python_cmake_module REQUIRED)

autoware_package()

find_package(BoostPython REQUIRED)

include_directories(
  ${BoostPython_INCLUDE_DIRS}

)
ament_python_install_package(${PROJECT_NAME})

set(CMAKE_SHARED_LIBRARY_PREFIX "")
ament_auto_add_library(_${PROJECT_NAME}_boost_python_utility SHARED
  src/utility.cpp
)
ament_auto_add_library(_${PROJECT_NAME}_boost_python_projection SHARED
  src/projection.cpp
)
ament_auto_add_library(_${PROJECT_NAME}_boost_python_regulatory_elements SHARED
  src/regulatory_elements.cpp
)
install(TARGETS _${PROJECT_NAME}_boost_python_utility _${PROJECT_NAME}_boost_python_projection _${PROJECT_NAME}_boost_python_regulatory_elements
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

ament_package()
