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

ament_add_gtest(test_locate_multisurface test_locate_multisurface.cpp)
target_link_libraries(test_locate_multisurface
  ${PROJECT_NAME}
)

ament_add_gtest(test_navmap_properties test_navmap_properties.cpp)
target_link_libraries(test_navmap_properties
  ${PROJECT_NAME}
)

ament_add_gtest(test_adjacency test_adjacency.cpp)
target_link_libraries(test_adjacency
  ${PROJECT_NAME}
)

ament_add_gtest(test_distance_field test_distance_field.cpp)
target_link_libraries(test_distance_field
  ${PROJECT_NAME}
)

ament_add_gtest(test_iterators test_iterators.cpp)
target_link_libraries(test_iterators
  ${PROJECT_NAME}
)

ament_add_gtest(test_layers_unknown_occupancy test_layers_unknown_occupancy.cpp)
target_link_libraries(test_layers_unknown_occupancy
  ${PROJECT_NAME}
)

ament_add_gtest(test_incremental test_incremental.cpp)
target_link_libraries(test_incremental
  ${PROJECT_NAME}
)
