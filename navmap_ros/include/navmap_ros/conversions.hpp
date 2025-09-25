// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef NAVMAP_ROS__CONVERSIONS_HPP_
#define NAVMAP_ROS__CONVERSIONS_HPP_

/**
 * @file conversions.hpp
 * @brief Conversions between the core NavMap representation and ROS messages.
 *
 * This header provides:
 *  - Lossless conversion between the core type `navmap::NavMap` and the compact
 *    transport message `navmap_ros_interfaces::msg::NavMap`.
 *  - Bidirectional conversion between `nav_msgs::msg::OccupancyGrid` and `navmap::NavMap`
 *    using a regular triangular mesh (shared vertices, two triangles per grid cell).
 *
 * ### Occupancy mapping
 * When building a NavMap from an OccupancyGrid, a per-NavCel (triangle) layer named
 * `"occupancy"` (uint8) is added with the following value mapping:
 *   - `-1` (unknown) → `255`
 *   - `0..100` (percent) → `0..254` (linear scaling)
 *
 * The reverse conversion reuses that layer when present.
 */

#include <string>
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "navmap_ros_interfaces/msg/nav_map.hpp"
#include "navmap_ros_interfaces/msg/nav_map_layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "navmap_core/NavMap.hpp"

/**
 * @namespace navmap_ros
 * @brief Conversions between core NavMap and ROS-level structures.
 */
namespace navmap_ros
{

// --------- NavMap <-> ROS message ---------

/**
 * @brief Convert a core `navmap::NavMap` into its compact ROS transport message.
 *
 * @param[in] nm Core NavMap to be serialized into a ROS message.
 * @return A `navmap_ros_interfaces::msg::NavMap` containing geometry (vertices, triangles),
 *         surfaces metadata and user-defined layers.
 *
 * @details
 *  - Intended to be **lossless** with respect to the information represented in the message.
 *  - The ordering of vertices/triangles/layers in @p nm is preserved in the resulting message.
 *
 * @note This function does not perform IO; it only builds the message in-memory.
 */
navmap_ros_interfaces::msg::NavMap to_msg(
  const navmap::NavMap & nm);

/**
 * @brief Reconstruct a core `navmap::NavMap` from the ROS transport message.
 *
 * @param[in] msg Input `navmap_ros_interfaces::msg::NavMap` message.
 * @return A core `navmap::NavMap` equivalent to the content of @p msg.
 *
 * @details
 *  - Intended to be the inverse of ::navmap_ros::to_msg for a round-trip without loss.
 *  - Assumes that the message is internally consistent (sizes and indices match).
 *
 * @throw std::runtime_error If the message describes inconsistent geometry or layer sizes.
 */
navmap::NavMap from_msg(const navmap_ros_interfaces::msg::NavMap & msg);

/**
 * @brief Convert a single layer from a NavMap into a ROS message.
 *
 * @param[in] nm     Input NavMap.
 * @param[in] layer  Name of the layer to export.
 * @return A NavMapLayer message containing the layer values and metadata.
 *
 * @details
 *  - The returned message contains the layer name, type tag, and exactly one populated data
 *    array whose length equals the number of NavCels in @p nm.
 *  - The function performs a type-safe extraction (U8/F32/F64).
 *
 * @throw std::runtime_error If the layer does not exist or has an unsupported type.
 */
navmap_ros_interfaces::msg::NavMapLayer to_msg(
  const navmap::NavMap & nm,
  const std::string & layer);

/**
 * @brief Import a single NavMapLayer message into a NavMap.
 *
 * If the layer already exists in @p nm, it is overwritten. Otherwise, it is created.
 * Performs type dispatch based on the message field `type`.
 *
 * @param[in] msg Input NavMapLayer message.
 * @param[in,out] nm  Destination NavMap (must already have navcels sized correctly).
 *
 * @details
 *  - The function verifies that the length of the populated data array matches
 *    the number of triangles (NavCels) in @p nm.
 *  - Exactly one of the arrays `data_u8`, `data_f32`, or `data_f64` must be set.
 *
 * @throw std::runtime_error If sizes are inconsistent or the message is ill-formed.
 */
void from_msg(
  const navmap_ros_interfaces::msg::NavMapLayer & msg,
  navmap::NavMap & nm);

// --------- OccupancyGrid <-> NavMap (shared vertices, per-NavCel layer) ---------

/**
 * @brief Build a `navmap::NavMap` from a `nav_msgs::msg::OccupancyGrid`
 *        using a regular triangular surface with shared vertices.
 *
 * @param[in] grid Input ROS OccupancyGrid (row-major, width×height, resolution and origin).
 * @return A core `navmap::NavMap` with:
 *   - Vertices: `(W+1) * (H+1)` laid on the grid plane, with `Z = grid.info.origin.position.z`.
 *   - Triangles: `2 * W * H` (two per cell), using diagonal pattern = 0.
 *   - One surface whose frame matches `grid.header.frame_id`, and grid metadata filled.
 *   - A per-NavCel layer named `"occupancy"` of type `uint8`, with values mapped as:
 *       `-1 → 255` (unknown), `0..100 → 0..254` (linear scaling).
 *
 * @details
 *  - Vertex layout follows the grid indexation with shared vertices across adjacent cells.
 *  - Triangle winding and diagonal split are deterministic (pattern = 0).
 *  - If `width == 0` or `height == 0`, the returned map contains no triangles.
 *
 * @note The grid origin pose may contain a rotation. The vertex Z is taken from the origin Z;
 *       handling of non-zero yaw/roll/pitch (if any) is implementation-defined in the builder.
 */
navmap::NavMap from_occupancy_grid(const nav_msgs::msg::OccupancyGrid & grid);

/**
 * @brief Convert a `navmap::NavMap` back to `nav_msgs::msg::OccupancyGrid`.
 *
 * @param[in] nm Core NavMap to be rasterized as an occupancy grid.
 * @return A ROS `OccupancyGrid` populated from @p nm.
 *
 * @details
 *  Two paths are considered:
 *  - **Fast exact path**: If the first surface encodes valid grid metadata (GridMeta) and
 *    there is a per-NavCel `"occupancy"` layer of type U8 with size `2 * W * H`, the function
 *    reconstructs an `OccupancyGrid` exactly (linear inverse mapping `0..254 → 0..100`,
 *    `255 → -1`).
 *  - **Generic fallback**: If the exact path is not applicable, the function samples cell
 *    centers via a navcel-locator (e.g., `locate_navcel`) and reads `"occupancy"` values to
 *    populate the grid.
 *
 * @note The fallback path assumes the presence of an `"occupancy"` layer. The precise sampling
 *       strategy (bounds, resolution, and handling of cells without a containing navcel) is
 *       implementation-defined.
 *
 * @warning If the map does not carry grid metadata or the `"occupancy"` layer is missing,
 *          the result may be incomplete or implementation-defined.
 */
nav_msgs::msg::OccupancyGrid to_occupancy_grid(const navmap::NavMap & nm);

/**
 * @brief Parameters controlling NavMap construction from unorganized points.
 *
 * These parameters guide neighborhood search, local meshing, and basic geometric filtering
 * used by the point-cloud based builders.
 */
struct BuildParams
{
  /** @brief Seed position (world frame) used by region growing or initial search heuristics. */
  Eigen::Vector3f seed = {0.0, 0.0, 0.0};

  /** @brief Target in-plane sampling resolution (meters) used by voxelization or gridding. */
  float resolution = 1.0;

  /** @brief Maximum allowed edge length (meters) when forming triangles. */
  float max_edge_len = 2.0;

  /** @brief Maximum allowed vertical jump (meters) between vertices of a triangle. */
  float max_dz = 0.25f;

  /** @brief Maximum slope with respect to the vertical axis (degrees). */
  float max_slope_deg = 30.0f;   // maximum slope w.r.t. vertical

  /** @brief Neighborhood radius (meters) for candidate connectivity. */
  float neighbor_radius = 2.0f;  // search radius

  /** @brief Alternative to radius: number of nearest neighbors (k-NN). */
  int   k_neighbors = 20;        // k-NN alternative to radius

  /** @brief Minimum triangle area (square meters) to reject degenerate faces. */
  float min_area = 1e-6f;        // minimum triangle area to avoid degenerates

  /** @brief If true, use radius-based neighborhoods; otherwise use k-NN. */
  bool  use_radius = true;

  /** @brief Minimum interior angle (degrees) to avoid sliver triangles. */
  float min_angle_deg = 20.0f;   // minimum interior angle (deg) to avoid sliver triangles
};

/**
 * @brief Build a NavMap surface from a PCL point cloud.
 *
 * @param[in] input_points Point set in world coordinates (`pcl::PointXYZ`).
 * @param[out] out_msg     Output transport message mirroring the created NavMap.
 * @param[in] params       Meshing and filtering parameters (see ::BuildParams).
 * @return The constructed `navmap::NavMap`.
 *
 * @details
 *  Typical steps implemented by this builder include:
 *  - Optional downsampling according to @p params.resolution.
 *  - Local neighborhood discovery using either radius (@p params.use_radius) or k-NN.
 *  - Edge and face filtering based on @p params.max_edge_len, @p params.min_area and
 *    @p params.min_angle_deg.
 *  - Optional slope gating using @p params.max_slope_deg with respect to the vertical axis.
 *  - Creation of a single surface with shared vertices and triangle indices.
 *  The function also fills @p out_msg with the compact ROS representation of the resulting map.
 *
 * @note Input is treated as an unorganized cloud. If normals or intensities are present,
 *       they are ignored by this overload.
 * @throw std::runtime_error If meshing fails due to inconsistent parameters or empty input.
 */
navmap::NavMap from_points(
  const  pcl::PointCloud<pcl::PointXYZ> & input_points,
  navmap_ros_interfaces::msg::NavMap & out_msg,
  BuildParams params);

/**
 * @brief Build a NavMap surface from a ROS `sensor_msgs::msg::PointCloud2`.
 *
 * @param[in] pc2      Input PointCloud2 message (expects fields `x`, `y`, `z`).
 * @param[out] out_msg Output transport message mirroring the created NavMap.
 * @param[in] params   Meshing and filtering parameters (see ::BuildParams).
 * @return The constructed `navmap::NavMap`.
 *
 * @details
 *  - The cloud is decoded to `pcl::PointXYZ` and processed as in ::navmap_ros::from_points.
 *  - Non-Cartesian fields present in @p pc2 are ignored by this overload.
 *  - The resulting NavMap is exported to @p out_msg for downstream publication or storage.
 *
 * @note If the message is empty or lacks the required fields, no geometry is produced.
 * @throw std::runtime_error If decoding fails or meshing cannot be completed.
 */
navmap::NavMap from_pointcloud2(
  const sensor_msgs::msg::PointCloud2 & pc2,
  navmap_ros_interfaces::msg::NavMap & out_msg,
  BuildParams params);

}  // namespace navmap_ros

#endif  // NAVMAP_ROS__CONVERSIONS_HPP_
