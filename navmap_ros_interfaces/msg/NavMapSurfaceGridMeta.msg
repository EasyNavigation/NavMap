# Optional metadata to allow exact, O(WH) round-trip with OccupancyGrid.

uint32 width
uint32 height
float32 resolution
geometry_msgs/Pose origin

# 0: tri0=(i,j)->(i+1,j)->(i+1,j+1), tri1=(i,j)->(i+1,j+1)->(i,j+1)
# 1: tri0=(i,j)->(i+1,j)->(i,j+1),   tri1=(i+1,j)->(i+1,j+1)->(i,j+1)
uint8 diag_pattern
