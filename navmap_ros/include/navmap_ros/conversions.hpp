// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3.0, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.


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

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <navmap_core/NavMap.hpp>
#include <navmap_ros_interfaces/msg/nav_map.hpp>

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
 */
navmap::NavMap from_msg(const navmap_ros_interfaces::msg::NavMap & msg);

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
 */
nav_msgs::msg::OccupancyGrid to_occupancy_grid(const navmap::NavMap & nm);

}  // namespace navmap_ros

#endif  // NAVMAP_ROS__CONVERSIONS_HPP_
