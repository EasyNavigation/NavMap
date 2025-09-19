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

ament_add_gtest(test_locate_grid test_locate_grid.cpp)
target_link_libraries(test_locate_grid
  ${PROJECT_NAME}
)

ament_add_gtest(test_raycast_many test_raycast_many.cpp)
target_link_libraries(test_raycast_many
  ${PROJECT_NAME}
)

ament_add_gtest(test_navmap_easy test_navmap_easy.cpp)
target_link_libraries(test_navmap_easy
  ${PROJECT_NAME}
)

ament_add_gtest(test_navmap_assign test_navmap_assign.cpp)
target_link_libraries(test_navmap_assign
  ${PROJECT_NAME}
)
