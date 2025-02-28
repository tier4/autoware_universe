cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_visualization)

find_package(autoware_cmake REQUIRED)

find_package(TinyXML2 CONFIG QUIET)
if(NOT TinyXML2_FOUND)
  find_path(TINYXML2_INCLUDE_DIR NAMES tinyxml2.h)
  find_library(TINYXML2_LIBRARY tinyxml2)
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(TinyXML2 DEFAULT_MSG TINYXML2_LIBRARY TINYXML2_INCLUDE_DIR)
  mark_as_advanced(TINYXML2_INCLUDE_DIR TINYXML2_LIBRARY)
  if(NOT TARGET tinyxml2::tinyxml2)
    add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
    set_property(TARGET tinyxml2::tinyxml2 PROPERTY IMPORTED_LOCATION ${TINYXML2_LIBRARY})
    set_property(TARGET tinyxml2::tinyxml2 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${TINYXML2_INCLUDE_DIR})
    list(APPEND TinyXML2_TARGETS tinyxml2::tinyxml2)
  endif()
endif()

autoware_package()

find_package(OpenCV REQUIRED)

ament_auto_add_library(traffic_light_roi_visualizer SHARED
  src/traffic_light_roi_visualizer/node.cpp
  src/traffic_light_roi_visualizer/shape_draw.cpp
)

target_include_directories(traffic_light_roi_visualizer SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(traffic_light_roi_visualizer
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(traffic_light_roi_visualizer
  PLUGIN "autoware::traffic_light::TrafficLightRoiVisualizerNode"
  EXECUTABLE traffic_light_visualization_node
)

ament_auto_add_library(traffic_light_map_visualizer SHARED
  src/traffic_light_map_visualizer/node.cpp
)

rclcpp_components_register_node(traffic_light_map_visualizer
  PLUGIN "autoware::traffic_light::TrafficLightMapVisualizerNode"
  EXECUTABLE traffic_light_map_visualizer_node
)

# Copy the assets directory to the installation directory
install(
    DIRECTORY images/
    DESTINATION share/${PROJECT_NAME}/images
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
