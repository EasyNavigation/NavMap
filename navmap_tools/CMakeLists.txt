cmake_minimum_required(VERSION 3.16)
project(navmap_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(navmap_core REQUIRED)
find_package(navmap_ros REQUIRED)
find_package(navmap_ros_interfaces REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree search)

# ---------------- Python package + CLI entry point ----------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/navmap_gis_tool
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------- C++ tools ----------------
add_executable(pointcloud_to_navmap src/pointcloud_to_navmap.cpp)
target_include_directories(pointcloud_to_navmap PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pointcloud_to_navmap
  navmap_core::navmap_core
  navmap_ros::navmap_ros
  ${navmap_ros_interfaces_TARGETS}
  ${PCL_LIBRARIES}
)

add_executable(navmap_publisher src/navmap_publisher.cpp)
target_link_libraries(navmap_publisher
  rclcpp::rclcpp
  navmap_core::navmap_core
  navmap_ros::navmap_ros
  ${navmap_ros_interfaces_TARGETS}
)

install(TARGETS
  pointcloud_to_navmap
  navmap_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(navmap_tools_pytest test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
