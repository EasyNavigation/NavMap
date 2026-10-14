ament_add_gtest(test_conversions test_conversions.cpp)
target_link_libraries(test_conversions
  ${PROJECT_NAME}
)

ament_add_gtest(test_navmap_io test_navmap_io.cpp)
target_link_libraries(test_navmap_io
  ${PROJECT_NAME}
)

ament_add_gtest(test_from_points test_from_points.cpp)
target_include_directories(test_from_points PRIVATE ${pcl_conversions_INCLUDE_DIRS})
target_link_libraries(test_from_points
  ${PROJECT_NAME}
  ${pcl_conversions_TARGETS}
  ${pcl_conversions_LIBRARIES}
)
