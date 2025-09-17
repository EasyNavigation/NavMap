# NavMap

NavMap is an open-source C++ and ROS 2 library for representing **navigable surfaces** for mobile robot navigation and localization.  
Unlike classic grid-based maps, NavMap stores the environment as **triangular meshes** (NavCels), enabling efficient queries and multi-surface environments (e.g., multi-floor buildings).

---

## ✨ Features

- **Triangular cell mesh representation** with adjacency relations.
- Dynamic runtime **layers**: per-cell or per-vertex attributes (occupancy, elevation, cost, traversability, etc.).
- **Locate API**: find the NavCel under/around a 3D position using BVH acceleration and raycasting.
- **Raytracing**: Möller–Trumbore intersection with a simple BVH for efficiency.
- **Multi-surface support**: naturally supports multiple disconnected surfaces (e.g., separate floors).

---

## 📂 Repository structure

This repository is organized into several ROS 2 packages:

- **`navmap_core/`**  
  Core C++ library implementing NavMap. Minimal dependencies (Eigen3).

- **`navmap_ros/`**  
  ROS 2 conversions and message definitions:  
  - `navmap::NavMap` ↔ `navmap_ros_interfaces::msg::NavMap`  
  - `nav_msgs::msg::OccupancyGrid` ↔ `navmap::NavMap`

- **`navmap_rviz_plugin/`**  
  RViz2 plugin for visualization of NavMap messages:  
  - Displays surfaces and layers.  
  - Optional per-cell normal rendering.  
  - Layer-based coloring.

- **`navmap_tools/`**  
  Tools and utilities for building and testing NavMaps (mesh import/export, conversions, etc.).

> _Add an overview diagram here_
```markdown
![Overview](docs/images/overview.png)
```

---

## ⚙️ Build instructions

NavMap can be built as a standalone C++ library or within a ROS 2 workspace.

### ROS 2 colcon build

```bash
# Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/<your-org>/NavMap.git

# Build
cd ~/ros2_ws
colcon build --packages-up-to navmap_core navmap_ros navmap_rviz_plugin navmap_tools

# Source workspace
source install/setup.bash
```

### Dependencies

- C++17 compiler
- [Eigen3](https://eigen.tuxfamily.org/)
- ROS 2 (tested with Humble, Iron, Jazzy)
- RViz2 (for the visualization plugin)
- PCL (for mesh construction utilities)

---

## 🚀 Usage (C++ API)

This section shows **small, self-contained snippets** that demonstrate how to create a `NavMap`, add geometry, attach layers, query values, and locate the triangle (NavCel) corresponding to a 3D position.
> **Note**: After modifying geometry (vertices, triangles, or surfaces), always call `rebuild_geometry_accels()` before performing queries such as `locate_navcel()` or `raycast()`.

---

## 1. Create a minimal NavMap (single square floor with two triangles)

```cpp
#include <navmap_core/NavMap.hpp>
#include <Eigen/Core>

using navmap::NavMap;
using navmap::NavCelId;

NavMap nm;

// Create a surface
std::size_t surf_idx = nm.create_surface("map");

// Add 4 vertices of a unit square (z=0)
uint32_t v0 = nm.add_vertex({0.f, 0.f, 0.f});
uint32_t v1 = nm.add_vertex({1.f, 0.f, 0.f});
uint32_t v2 = nm.add_vertex({1.f, 1.f, 0.f});
uint32_t v3 = nm.add_vertex({0.f, 1.f, 0.f});

// Add 2 triangles
NavCelId c0 = nm.add_navcel(v0, v1, v2);
NavCelId c1 = nm.add_navcel(v0, v2, v3);

// Assign them to the surface
nm.add_navcel_to_surface(surf_idx, c0);
nm.add_navcel_to_surface(surf_idx, c1);

// Build normals, adjacency, BVH, etc.
nm.rebuild_geometry_accels();
```

---

## 2. Add a per-NavCel layer (cost or occupancy)

```cpp
// Add a layer of type float called "cost"
auto cost = nm.add_layer<float>("cost", "Traversal cost", "" , 0.0f);

// Assign values to each triangle
nm.layer_set<float>("cost", c0, 1.0f);
nm.layer_set<float>("cost", c1, 5.0f);
```

---

## 3. Read a layer value for a given NavCel

```cpp
double v = nm.layer_get_as_double("cost", c0);  // → 1.0
```

---

## 4. Locate the NavCel corresponding to a 3D position

```cpp
size_t surf_idx;
NavCelId cid;
Eigen::Vector3f bary, hit;

bool ok = nm.locate_navcel(Eigen::Vector3f(0.5f, 0.5f, 0.1f),
                           surf_idx, cid, bary, &hit);

if (ok) {
  std::cout << "Point belongs to surface " << surf_idx
            << ", NavCel " << cid
            << " with barycentric coords " << bary.transpose() << std::endl;
}
```

---

## 5. Sample a layer at a world position

```cpp
double val = nm.sample_layer_at("cost", Eigen::Vector3f(0.2f, 0.8f, 1.0f), -1.0);
if (!std::isnan(val)) {
  std::cout << "Cost at (0.2,0.8) is " << val << std::endl;
}
```

If the layer does not exist or the position cannot be located, the fallback value (`-1.0` here) is returned.

---

## 6. Raycasting

```cpp
NavCelId hit_cid;
float t;
Eigen::Vector3f hit_pt;

bool hit = nm.raycast(
  Eigen::Vector3f(0.5f, 0.5f, 2.0f),   // origin
  Eigen::Vector3f(0.0f, 0.0f, -1.0f),  // direction
  hit_cid, t, hit_pt);

if (hit) {
  std::cout << "Ray hit NavCel " << hit_cid
            << " at " << hit_pt.transpose() << std::endl;
}
```

---

## 7. Serialize to / from ROS messages

Conversion functions are provided in `navmap_ros`:

```cpp
#include <navmap_ros/conversions.hpp>
#include <navmap_ros_interfaces/msg/nav_map.hpp>

// Convert to ROS message
navmap_ros_interfaces::msg::NavMap msg = navmap_ros::to_msg(nm);

// Convert back to core structure
navmap::NavMap nm2 = navmap_ros::from_msg(msg);
```

---

## 8. Save and load from disk

NavMap supports saving and loading using YAML + mesh files:

```cpp
#include <navmap_ros/map_io.hpp>

// Save NavMap to disk
navmap_ros::saveMapToFile(nm, "/tmp/navmap.yaml");

// Load NavMap from disk
navmap::NavMap nm3 = navmap_ros::loadMapFromYaml("/tmp/navmap.yaml");
```

---

## 9. Classic low-level API

For advanced control you can still access internal data directly:

```cpp
// Access vertex positions
Eigen::Vector3f v = nm.positions.at(0);

// Iterate over NavCels
for (const auto & cel : nm.navcels) {
  Eigen::Vector3f centroid = nm.navcel_centroid(&cel - &nm.navcels[0]);
  std::cout << "NavCel area: " << cel.area
            << " centroid: " << centroid.transpose() << std::endl;
}
```

---

With this combination of **easy-to-use high-level methods** and the **classic API**, you can build and query navigation meshes both quickly and with full control when needed.

---

## 🧪 Testing

NavMap provides unit tests with GTest. To run them:

```bash
colcon test --packages-select navmap_core navmap_ros
colcon test-result --verbose
```

---

## 🤝 Contributing

Contributions are welcome! Please open issues and pull requests on GitHub.  
Before submitting code, run the linters and tests:

```bash
colcon test
ament_lint_auto
```

---

## 📜 License

This project is licensed under the **GNU General Public License v3.0 (GPL-3.0)**.  
See the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgements

Developed at the **Intelligent Robotics Lab (Universidad Rey Juan Carlos)**.  
Part of the Easy Navigation (EasyNav) project.
