cmake_minimum_required(VERSION 3.14)
project(tier4_localization_launch)

find_package(autoware_cmake REQUIRED)
autoware_package()

install(PROGRAMS scripts/rviz2_with_filtered_ament.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  rviz
)
