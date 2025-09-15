# Compact NavMap transport.
std_msgs/Header header

# Vertex positions (SoA)
float32[] positions_x
float32[] positions_y
float32[] positions_z

# Optional per-vertex colors (RGBA). Fill only if has_vertex_rgba=true.
bool has_vertex_rgba
uint8[] colors_r
uint8[] colors_g
uint8[] colors_b
uint8[] colors_a

# Global triangle index buffers
uint32[] navcels_v0
uint32[] navcels_v1
uint32[] navcels_v2

# Surfaces (each lists indices into the global triangle arrays)
navmap_ros_interfaces/NavMapSurface[] surfaces

# Per-NavCel layers
navmap_ros_interfaces/NavMapLayer[] layers
