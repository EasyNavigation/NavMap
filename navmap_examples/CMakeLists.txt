cmake_minimum_required(VERSION 3.16)
project(navmap_examples)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(nav_msgs)
find_package(navmap_core REQUIRED)
find_package(navmap_ros REQUIRED)
find_package(navmap_ros_interfaces REQUIRED)


file(GLOB CORE_SOURCES "src/*.cpp")
foreach(src ${CORE_SOURCES})
  get_filename_component(name ${src} NAME_WE)
  add_executable(${name} ${src})
  target_link_libraries(${name} PUBLIC
    navmap_core::navmap_core
    navmap_ros::navmap_ros
    ${navmap_ros_interfaces_TARGETS}
  )
  install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
endforeach()

# ---------------- ROS2 examples ----------------
if(BUILD_ROS_EXAMPLES)
  if(NOT rclcpp_FOUND OR NOT nav_msgs_FOUND)
    find_package(rclcpp REQUIRED)
    find_package(nav_msgs REQUIRED)
  endif()

  include_directories(${NAVMAP_ROS_INCLUDE_DIR})

  file(GLOB ROS_SOURCES "src_ros2/*.cpp")
  foreach(src ${ROS_SOURCES})
    get_filename_component(name ${src} NAME_WE)
    add_executable(${name} ${src})
    target_link_libraries(${name} PUBLIC
      rclcpp::rclcpp
      navmap_core::navmap_core
      navmap_ros::navmap_ros
      ${navmap_ros_interfaces_TARGETS}
    )
    install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
  endforeach()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
