cmake_minimum_required(VERSION 3.14)
project(autoware_data_driven_planning_simulator)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/autoware/data_driven_planning_simulator/data_driven_planning_simulator_node.hpp
  include/autoware/data_driven_planning_simulator/models/baseline_model.hpp
  include/autoware/data_driven_planning_simulator/models/model_interface.hpp
  include/autoware/data_driven_planning_simulator/visibility_control.hpp
  src/data_driven_planning_simulator/data_driven_planning_simulator_node.cpp
  src/data_driven_planning_simulator/models/baseline_model.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::simulator::data_driven_planning_simulator::DataDrivenPlanningSimulator"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/evaluate_model.py
  scripts/export_model.py
  scripts/extract_dataset.py
  scripts/train_model.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_baseline_model
    test/test_baseline_model.cpp
  )
  target_link_libraries(test_baseline_model ${PROJECT_NAME})

  ament_add_pytest_test(test_dataset_tools
    test/test_dataset_tools.py
  )
endif()

ament_auto_package(INSTALL_TO_SHARE config docs launch param test)
