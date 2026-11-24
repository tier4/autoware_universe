cmake_minimum_required(VERSION 3.17)
project(autoware_ml_planner_preprocessing)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(xtensor REQUIRED)
find_package(t4scene 0.0.0 EXACT CONFIG REQUIRED)

set(ros_dependencies
  autoware_lanelet2_extension
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_utils_math
  lanelet2_core
)
foreach(dependency IN LISTS ros_dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/preprocessing/items/ego_shape.cpp
  src/preprocessing/items/map.cpp
  src/preprocessing/items/traffic_light_features.cpp
  src/preprocessing/preprocessing_utils.cpp
  src/preprocessing/scene_input_builder.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${ros_dependencies})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen t4scene::data xtensor)
target_link_libraries(${PROJECT_NAME} PRIVATE Boost::boost)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 xtensor t4scene ${ros_dependencies})

option(AUTOWARE_ML_PLANNER_PREPROCESSING_PYTHON "Install the T4Scene protobuf/NumPy interface" OFF)
if(AUTOWARE_ML_PLANNER_PREPROCESSING_PYTHON)
  if(NOT TARGET t4scene::protobuf_maps)
    message(FATAL_ERROR "The Python interface needs t4scene built with -DT4SCENE_WITH_MAPS=ON")
  endif()
  find_package(ament_cmake_python REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
  find_package(pybind11_vendor REQUIRED)
  find_package(pybind11 REQUIRED)
  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})
  ament_get_python_install_dir(python_install_dir)
  pybind11_add_module(_native MODULE src/python.cpp)
  target_link_libraries(_native PRIVATE ${PROJECT_NAME} t4scene::protobuf_maps)
  # Lanelet2 discovers Autoware regulatory elements through static registrars.
  # Keep their library loaded even when the binding has no direct symbol reference.
  set(lanelet2_registrars ${autoware_lanelet2_extension_LIBRARIES})
  list(FILTER lanelet2_registrars INCLUDE REGEX "(^|[:/])(lib)?autoware_lanelet2_extension_lib(\\.|$)")
  if(NOT lanelet2_registrars)
    message(FATAL_ERROR "autoware_lanelet2_extension did not export its regulatory element library")
  endif()
  target_link_libraries(_native PRIVATE
    "-Wl,--push-state,--no-as-needed"
    ${lanelet2_registrars}
    "-Wl,--pop-state"
  )
  install(TARGETS _native LIBRARY DESTINATION ${python_install_dir}/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scene_input_builder test/scene_input_builder_test.cpp)
  if(TARGET test_scene_input_builder)
    target_link_libraries(test_scene_input_builder ${PROJECT_NAME})
  endif()
endif()

ament_package()
