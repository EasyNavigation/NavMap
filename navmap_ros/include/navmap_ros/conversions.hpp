// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
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

#include <string>
#include <memory>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <navmap_core/NavMap.hpp>
#include <navmap_ros_interfaces/msg/nav_map.hpp>


namespace navmap_ros
{

// --------- NavMap <-> ROS message ---------

/** Convert core NavMap to ROS message (compact transport). */
navmap_ros_interfaces::msg::NavMap to_msg(
  const navmap::NavMap & nm,
  const std::string & frame_id);

/** Convert ROS message back to core NavMap. */
navmap::NavMap from_msg(const navmap_ros_interfaces::msg::NavMap & msg);

// --------- OccupancyGrid <-> NavMap (shared vertices, per-NavCel layer) ---------

/**
 * @brief Build a NavMap from an OccupancyGrid using a regular mesh with shared vertices.
 *
 * - Vertices: (W+1)*(H+1), Z=origin.z
 * - Triangles: 2*W*H, diagonal pattern=0
 * - One surface with frame_id = grid.header.frame_id and GridMeta filled
 * - Adds per-NavCel layer "occupancy" (uint8), mapping:
 *     -1 -> 255 (unknown), 0..100 -> 0..254
 */
navmap::NavMap from_occupancy_grid(const nav_msgs::msg::OccupancyGrid & grid);

/**
 * @brief Convert a NavMap back to OccupancyGrid.
 *
 * Fast path (exact): if first surface has GridMeta and layer "occupancy" (U8) sized 2*W*H.
 * Generic fallback: sample cell centers with locate_navcel() and read "occupancy".
 */
nav_msgs::msg::OccupancyGrid to_occupancy_grid(const navmap::NavMap & nm);

}  // namespace navmap_ros

#endif  // NAVMAP_ROS__CONVERSIONS_HPP_
