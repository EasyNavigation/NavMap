ament_add_gtest(test_conversions test_conversions.cpp)
target_link_libraries(test_conversions
  ${PROJECT_NAME}
)

ament_add_gtest(test_navmap_io test_navmap_io.cpp)
target_link_libraries(test_navmap_io
  ${PROJECT_NAME}
)
