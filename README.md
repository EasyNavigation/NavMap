
# NavMap (prototype)

A C++ core library to represent navigable surfaces for robot navigation and localization, with:
- Triangular cells called **NavCel** (3 vertex polygons).
- Dynamic, runtime **layers** (per-vertex attributes such as cost, traversability, occupancy).
- Fast **adjacency** (neighbors per NavCel in O(1)).
- **Raycasting** (Möller–Trumbore) accelerated by a simple BVH.
- **Locate** API to determine which NavCel lies under/around a 3D position (with hint walking + vertical raycast).
- Multi-surface support (e.g., multi-floor buildings).

This repo contains:
- `navmap_core/`: Header-only-friendly C++ library (plus .cpp) with minimal dependencies (Eigen3 for math).
- `navmap_ros/`: ROS 2 messages and stubs (optional; not required to build core).
- `navmap_tools/`: Placeholder for converters/tools.

## Build (navmap_core only)

```bash
mkdir -p build && cd build
cmake ../navmap_core -DCMAKE_BUILD_TYPE=Release
make -j
ctest --output-on-failure
```

Requirements:
- C++17
- Eigen3
- GTest (for tests)

## Notes
- This is a prototype for discussion and iteration. The BVH is minimal but functional.
- The ROS 2 packages include message definitions and stubs, but you may need to integrate into your ROS 2 workspace and adjust package metadata.
