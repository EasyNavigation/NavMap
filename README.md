# NavMap
[![Doxygen Deployment](https://github.com/EasyNavigation/NavMap/actions/workflows/doxygen-doc.yml/badge.svg)](https://github.com/EasyNavigation/NavMap/actions/workflows/doxygen-doc.yml)
[![rolling](https://github.com/EasyNavigation/NavMap/actions/workflows/rolling.yaml/badge.svg?branch=rolling)](https://github.com/EasyNavigation/NavMap/actions/workflows/rolling.yaml)
[![lyrical](https://github.com/EasyNavigation/NavMap/actions/workflows/lyrical.yaml/badge.svg?branch=lyrical)](https://github.com/EasyNavigation/NavMap/actions/workflows/lyrical.yaml)
[![kilted](https://github.com/EasyNavigation/NavMap/actions/workflows/kilted.yaml/badge.svg?branch=kilted)](https://github.com/EasyNavigation/NavMap/actions/workflows/kilted.yaml)
[![jazzy](https://github.com/EasyNavigation/NavMap/actions/workflows/jazzy.yaml/badge.svg?branch=jazzy)](https://github.com/EasyNavigation/NavMap/actions/workflows/jazzy.yaml)
[![humble](https://github.com/EasyNavigation/NavMap/actions/workflows/humble.yaml/badge.svg?branch=humble)](https://github.com/EasyNavigation/NavMap/actions/workflows/humble.yaml)

<img width="400" alt="navmap_logo_white" src="https://github.com/user-attachments/assets/470f45cc-3142-4c51-8375-2446d282d4de" />

[![NavMap SLAM](https://img.youtube.com/vi/t5p6c2HO7UA/0.jpg)](https://www.youtube.com/watch?v=t5p6c2HO7UA)

NavMap is an open-source C++ and ROS 2 library for representing **navigable surfaces** for mobile robot navigation and localization.
Unlike classic grid-based maps, NavMap stores the environment as **triangular meshes** (NavCels), enabling efficient queries and multi-surface environments (e.g., multi-floor buildings).

--- 

<img width="240" height="47" alt="MICIU+Cofinanciado+AEI" src="https://github.com/user-attachments/assets/fedfcf13-6af6-43d7-8290-fe22da4e1db0" />
<img width="240" height="50" alt="eu_funded_en" src="https://github.com/user-attachments/assets/b11da974-9201-4f79-902e-c9c20e8aa7a4" />
Funded by the European Union through the Horizon Europe programme under Grant Agreement No. 101070254 (CoreSense), and by MICIU/AEI/10.13039/501100011033 and ERDF/EU under grants PERMAP PID2024-161761OB-C21 and PLANNAV PID2024-161761OB-C22 (AURORAS).

---

## ✨ Features

- **Triangular cell mesh representation** with adjacency relations.
- Dynamic runtime **layers**: per-cell or per-vertex attributes (occupancy, elevation, cost, traversability, etc.).
- **Locate API**: find the NavCel under/around a 3D position using BVH acceleration and raycasting.
- **Raytracing**: Möller–Trumbore intersection with a simple BVH for efficiency.
- **Multi-surface support**: naturally supports multiple disconnected surfaces (e.g., separate floors).
- **Unknown values and probabilistic occupancy**: each layer can declare the value that means "not observed"; `Occupancy.hpp` keeps log-odds occupancy (OctoMap's clamped update) in an int16 layer, with free / occupied / unknown states.
- **Iterators** over NavCels (`navmap_core/Iterators.hpp`): box, disc, ring, polygon, distance order and segment walk; regions grow over the surface, so they never jump to another floor.

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
  Tools and utilities for building and testing NavMaps (mesh import/export, conversions, etc.)

- **`navmap_examples/`**  
  Practical examples demonstrating the usage of NavMap, both core C++ API and ROS 2 integrations.

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
colcon build --packages-up-to navmap_core navmap_ros navmap_rviz_plugin navmap_tools navmap_examples

# Source workspace
source install/setup.bash
```

### Dependencies

- C++23 compiler
- [Eigen3](https://eigen.tuxfamily.org/)
- ROS 2 (tested with Humble, Iron, Jazzy)
- RViz2 (for the visualization plugin)
- PCL (for mesh construction utilities)

---

## 🚀 Usage (C++ API)

[C++ API](https://easynavigation.github.io/NavMap/)

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
double v = nm.layer_get<double>("cost", c0);  // → 1.0
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

## 7. Marking areas (`set_area`)

You can set values over regions of the map using shapes such as **circular** or **rectangular** areas:

```cpp
// Add an occupancy layer
nm.add_layer<uint8_t>("occupancy", "Occupancy", "%", 0);

// Mark a circular area at (0.5,0.5) with radius 0.3
nm.set_area<uint8_t>(Eigen::Vector3f(0.5f, 0.5f, 0.0f),
                     (uint8_t)254,
                     "occupancy",
                     navmap::AreaShape::CIRCULAR,
                     0.3f);

// Mark a rectangular area centered at (0.8,0.2)
nm.set_area<uint8_t>(Eigen::Vector3f(0.8f, 0.2f, 0.0f),
                     (uint8_t)200,
                     "occupancy",
                     navmap::AreaShape::RECTANGULAR,
                     0.35f);
```

---

## 7b. Iterating over NavCels (`Iterators.hpp`)

Visitors are called once per NavCel; returning `false` stops. Regions around a point grow from the NavCel under it through the mesh adjacency, so a disc on one floor never reaches the floor above. Reuse a `TraversalScratch` to avoid allocations.

```cpp
#include <navmap_core/Iterators.hpp>

navmap::TraversalScratch scratch;
const Eigen::Vector3f robot(2.0f, 1.0f, 0.0f);

// NavCels within 0.5 m of the robot (3D distance to the centroid)
navmap::for_each_in_disc(nm, robot, 0.5f, [&](navmap::NavCelId c) {/* ... */},
  navmap::Metric::EUCLIDEAN, &scratch);

// NavCels crossed by a sensor ray projected on the surface, with the entry/exit parameters
navmap::for_each_on_segment(nm, robot, Eigen::Vector3f(6.0f, 3.0f, 0.0f),
  [&](navmap::NavCelId c, float t_in, float t_out) {/* clear c */});

// Nearest NavCel along the surface (geodesic order) with a free value
navmap::for_each_by_distance(nm, seed, robot, 3.0f, [&](navmap::NavCelId c, float d) {
    return nm.layer_get<uint8_t>("occupancy", c) != 0;  // false stops at the first free one
  });
```

Also `for_each_in_aabb` (every surface, via the BVH), `for_each_in_ring` and `for_each_in_polygon` (XY polygon on the seed's floor). `Metric::GEODESIC` measures along the surface and bends around borders; it is about 80 times costlier than `EUCLIDEAN`, so use it for occasional queries.

---

## 7c. Unknown values and log-odds occupancy (`Occupancy.hpp`)

```cpp
#include <navmap_core/Occupancy.hpp>

nm.add_layer<float>("elevation", "terrain height", "m", 0.0f);
nm.set_layer_unknown("elevation", std::numeric_limits<double>::quiet_NaN());  // NaN = not surveyed
bool unseen = nm.is_unknown("elevation", cid);

navmap::OccupancyLayer occ(nm);   // int16 layer "occupancy_logodds", all unknown
occ.miss(crossed_cid);            // a ray crossed it
occ.hit(end_cid);                 // a ray ended on it
if (occ.state(cid) == navmap::OccupancyState::OCCUPIED) {/* ... */}
```

The unknown value, description and unit of each layer travel in the message and in the file. Which NavCels a ray hits or crosses is up to the algorithm using it (see `for_each_on_segment`). `ros2 launch navmap_examples unknown_occupancy.launch.py` shows both in RViz2 (build with `-DBUILD_ROS_EXAMPLES=ON`).

---

## 7d. Incremental updates (change tracking, `DistanceField.hpp`)

Layers record which NavCels changed since the last `clear_changes()` (`layer_set`, `OccupancyLayer`; bulk writes mark the whole layer), so a consumer only reprocesses what changed. `DistanceField` keeps a distance-to-seed field up to date from the seed cells added and removed:

```cpp
#include <navmap_core/DistanceField.hpp>

navmap::DistanceField df(nm, /*from_boundary=*/true, /*max_distance=*/0.7f);
df.compute(lethal_cells);                       // once
df.update(newly_lethal, no_longer_lethal);      // every cycle: cost of the area that changes
for (navmap::NavCelId c : df.changed()) {/* recompute the cost of c from df[c] */}
```

With a finite `max_distance` the field is exact (straight distance to the nearest seed reaching the NavCel over the surface), and identical after any sequence of updates to a full recomputation.

---

## 7e. Geometric descriptors (`Descriptors.hpp`)

`navmap::compute_descriptors(nm)` adds F32 layers per NavCel: `slope` (deg), `roughness` (RMS distance to the plane fitted in a disc, m), `step` (height span in a small disc, m) and `clearance` (free height to the next surface of the map above, m; +inf if none). Discs grow over the surface, so floors never mix. They only describe the surface: weighing them into a cost is up to the planner.

---

## 7f. Links between surfaces (`SurfaceLink`)

Stairs, elevators or doors between NavCels that do not share an edge. The map only records them; whether a robot can take one, and at which cost, is up to the planner. They travel in the message and the file, and RViz2 draws them (*Draw Links*).

```cpp
nm.add_link(ground_cid, upper_cid, navmap::LinkType::ELEVATOR);            // both ways
nm.add_link(top_cid, bottom_cid, navmap::LinkType::STAIRS, 4.5f, false);   // one way, 4.5 m
for (std::uint32_t i : nm.links_from(cid)) {const auto & l = nm.links()[i]; /* ... */}
```

---

## 7g. Combining layers (`Combine.hpp`)

Each source keeps its own layer (a static prior, obstacles from a sensor, a keep-out zone), and a master layer is composed from them with `navmap::combine_layer(nm, dst, src, op, options)`. The operators are those of costmap_2d: `OVERWRITE`, `TRUE_OVERWRITE`, `MAX`, `MAX_KEEP_UNKNOWN`, `MIN` and `ADD` (saturating at `options.cap`). Each layer's declared unknown value is respected, types may differ, `options.cells` restricts the work to the cells that changed, and `options.provenance` names a U8 layer that records which source set each cell. Clearing the sensor layer never erases the prior, because the sensor never writes it. The cost values (`navmap_core/Costs.hpp`) are those of costmap_2d.

```cpp
#include <navmap_core/Combine.hpp>

navmap::CombineOptions o;
o.provenance = "source";
o.source_id = 1;
navmap::combine_layer(nm, "cost", "prior", navmap::CombineOp::TRUE_OVERWRITE, o);
o.source_id = 2;
o.cells = nm.layers.get("obstacles")->changed();
navmap::combine_layer(nm, "cost", "obstacles", navmap::CombineOp::MAX, o);
```

---

## 7h. Simplification that keeps the layers (`Simplify.hpp`)

`navmap::simplify(nm, params)` merges NavCels where the surface is flat (within `max_error`) and the layers have the same value, and keeps the original ones where they change: every layer keeps its value at every point, open edges keep their shape and the topology does not change (floors, holes, links). `max_edge_length` bounds the NavCel size. Layers that are per-cell geometry (`elevation`, `support` from `from_points`) go in `ignore_layers`.

```bash
ros2 run navmap_tools navmap_simplify in.navmap out.navmap --max-error 0.02 \
  --ignore-layer elevation --ignore-layer support
```

| Map (playgrounds) | NavCels | `max_error` | After | File (zlib) |
|---|---|---|---|---|
| `home2_20cm` (occupancy grid) | 10 716 | 0.02 m | 22.6 % | 248 KB → 16 KB |
| `warehouse_20cm` (occupancy grid) | 18 720 | 0.02 m | 21.4 % | 433 KB → 25 KB |
| `parking_garage` (`from_points`) | 38 543 | 0.02 / 0.1 m | 10.6 / 4.2 % | 1.06 MB → 56 / 24 KB |
| `excavation_urjc` (`from_points`) | 29 728 | 0.02 / 0.1 m | 67 / 38 % | 822 KB → 235 / 142 KB |

---

## 8. Serialize to / from ROS messages

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

### Layer updates

`navmap_ros::to_layer_update_msg(nm, "obstacles")` publishes only what changed since the layer's last `clear_changes()`: a sparse `NavMapLayer` (`indices` + values) when few NavCels changed, the full layer otherwise. Every `NavMap` and `NavMapLayer` message carries the map's `geometry_id`, so `from_msg` (and RViz2) reject updates meant for another geometry.

---

## 9. Save and load from disk

```cpp
#include <navmap_ros/navmap_io.hpp>

navmap_ros::io::save_to_file(nm, "/tmp/map.navmap");     // zlib-compressed by default
navmap::NavMap nm3;
navmap_ros::io::load_from_file("/tmp/map.navmap", nm3);
```

---

> **File format.** `.navmap` files are version 2: the CDR-serialized `NavMap` message (layer metadata, int16 layers, links, geometry id), zlib-compressed by default (`SaveOptions::compression`, `level`). zlib is available on every ROS distro and in the Python standard library. Version 1 files, and files without header, still load: they are migrated with a deprecation warning and rewritten in place when the location is writable.

## 10. Classic low-level API

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

## 📦 Usage (Examples package)

In addition to the snippets above, the repository provides the package `navmap_examples` with ready-to-run executables.

### Build

```bash
colcon build --packages-select navmap_examples
```

If your `navmap_core` / `navmap_ros` do not yet export CMake targets, you can provide include paths:

```bash
colcon build --packages-select navmap_examples   --cmake-args -DNAVMAP_CORE_INCLUDE_DIR=~/ros2_ws/src/NavMap/navmap_core/include                -DNAVMAP_ROS_INCLUDE_DIR=~/ros2_ws/src/NavMap/navmap_ros/include
```

Disable ROS 2 examples:

```bash
colcon build --packages-select navmap_examples --cmake-args -DBUILD_ROS_EXAMPLES=OFF
```

### Run core examples

```bash
ros2 run navmap_examples 01_flat_plane
ros2 run navmap_examples 02_two_floors
ros2 run navmap_examples 03_slope_surface
ros2 run navmap_examples 04_layers
ros2 run navmap_examples 05_neighbors_and_centroids
ros2 run navmap_examples 06_area_marking
ros2 run navmap_examples 07_raycast
ros2 run navmap_examples 08_copy_and_assign
```

Each demonstrates a different feature: geometry creation, multi-surface, U8/F32 layers, centroids and neighbors, area marking, raycasting, and copy/assign semantics.

### Run ROS 2 examples

If `BUILD_ROS_EXAMPLES=ON`:

```bash
ros2 run navmap_examples 01_from_occgrid
ros2 run navmap_examples 02_to_occgrid
ros2 run navmap_examples 03_save_load
```

Or via launch:

```bash
ros2 launch navmap_examples navmap_to_occgrid.launch.py
```

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

This project is licensed under the **Apache 2 License**.  
See the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgements

Developed at the **Intelligent Robotics Lab (Universidad Rey Juan Carlos)**.  
Part of the Easy Navigation (EasyNav) project.
