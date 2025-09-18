cmake_minimum_required(VERSION 3.10)
project(navmap_rviz_plugin)

# 1) Qt y AUTOMOC
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(navmap_ros_interfaces REQUIRED)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

qt5_wrap_cpp(NAVMAP_MOC_SRCS
  include/navmap_rviz_plugin/NavMapDisplay.hpp
)

add_library(${PROJECT_NAME} SHARED
  src/NavMapDisplay.cpp
  include/navmap_rviz_plugin/NavMapDisplay.hpp
  ${NAVMAP_MOC_SRCS}
)
target_link_libraries(navmap_rviz_plugin PUBLIC
  ${navmap_ros_interfaces_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  Qt5::Core
  Qt5::Widgets
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
  Qt5
  navmap_ros_interfaces
  geometry_msgs
  std_msg
)
ament_package()