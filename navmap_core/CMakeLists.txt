
cmake_minimum_required(VERSION 3.14)
project(navmap_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/NavMap.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
)

install(
  DIRECTORY include/
  DESTINATION include/
)

option(NAVMAP_BUILD_BENCH "Build the navmap_core micro-benchmark" OFF)
if(NAVMAP_BUILD_BENCH)
  add_executable(navmap_bench bench/navmap_bench.cpp)
  target_link_libraries(navmap_bench ${PROJECT_NAME})
  install(TARGETS navmap_bench RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Humble's ament_copyright does not recognize the license with the EasyNav line in the header
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  endif()
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_subdirectory(tests)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  Eigen3
)
ament_package()
