ament_add_gtest(test_navmap test_navmap.cpp)
target_link_libraries(test_navmap
  ${PROJECT_NAME}
)

ament_add_gtest(test_geometry test_geometry.cpp)
target_link_libraries(test_geometry
  ${PROJECT_NAME}
)


ament_add_gtest(test_navmap_uniform_and_closest test_navmap_uniform_and_closest.cpp)
target_link_libraries(test_navmap_uniform_and_closest
  ${PROJECT_NAME}
)
