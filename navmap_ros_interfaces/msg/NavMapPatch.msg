# Changes to a NavMap already sent: the map with geometry_id = base_geometry_id becomes the one
# with geometry_id. A receiver whose map is not the base ignores it and waits for a full NavMap.
#
# Vertices are only added (from first_vertex on). The NavCels in navcel_indices are rewritten
# (vertices and the value of every layer); the map ends with navcels_size NavCels (new ones not
# listed are degenerate: vertices 0, 0, 0). Surfaces in surface_indices are replaced. Neighbours
# are not sent (as in NavMap): a receiver that needs them rebuilds the adjacency.

std_msgs/Header header

uint64 base_geometry_id
uint64 geometry_id

uint32 first_vertex
float32[] positions_x
float32[] positions_y
float32[] positions_z

# Colours of the new vertices, when the map has vertex colours
uint8[] colors_r
uint8[] colors_g
uint8[] colors_b
uint8[] colors_a

uint32 navcels_size
uint32[] navcel_indices
uint32[] navcels_v0
uint32[] navcels_v1
uint32[] navcels_v2

# Every layer, sparse: indices = navcel_indices
navmap_ros_interfaces/NavMapLayer[] layers

uint32[] surface_indices
navmap_ros_interfaces/NavMapSurface[] surfaces

# All the links of the map (they are few)
navmap_ros_interfaces/NavMapLink[] links
