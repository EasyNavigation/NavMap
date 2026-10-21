ament_add_gtest(test_conversions test_conversions.cpp)
target_link_libraries(test_conversions
  ${PROJECT_NAME}
)

ament_add_gtest(test_navmap_io test_navmap_io.cpp)
target_link_libraries(test_navmap_io
  ${PROJECT_NAME}
)

# Coverage builds (CI) are slow: the mesher and viewpoint tests take over a minute there
ament_add_gtest(test_from_points test_from_points.cpp TIMEOUT 300)
target_include_directories(test_from_points PRIVATE ${pcl_conversions_INCLUDE_DIRS})
target_link_libraries(test_from_points
  ${PROJECT_NAME}
  ${pcl_conversions_TARGETS}
  ${pcl_conversions_LIBRARIES}
)

ament_add_gtest(test_layer_metadata_and_migration test_layer_metadata_and_migration.cpp)
target_link_libraries(test_layer_metadata_and_migration
  ${PROJECT_NAME}
)
