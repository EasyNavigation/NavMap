cmake_minimum_required(VERSION 3.10)
project(navmap_rviz_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(navmap_ros_interfaces REQUIRED)
find_package(navmap_ros REQUIRED)
find_package(navmap_core REQUIRED)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(${PROJECT_NAME} SHARED
  src/navmap_rviz_plugin/NavMapDisplay.cpp
  src/navmap_rviz_plugin/navmap_goal_tool.cpp
  src/navmap_rviz_plugin/navmap_pose_tool.cpp
  include/navmap_rviz_plugin/NavMapDisplay.hpp
  include/navmap_rviz_plugin/navmap_goal_tool.hpp
  include/navmap_rviz_plugin/navmap_pose_tool.hpp
)
target_link_libraries(navmap_rviz_plugin PUBLIC
  ${navmap_ros_interfaces_TARGETS}
  navmap_ros::navmap_ros
  navmap_core::navmap_core
  pluginlib::pluginlib
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(rviz_common resource/navmap_rviz_plugin_description.xml)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

install(FILES resource/navmap_rviz_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  pluginlib
  rviz_common
  rviz_rendering
  rviz_default_plugins
  navmap_ros_interfaces
  geometry_msgs
  std_msg
)
ament_package()