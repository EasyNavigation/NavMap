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


#ifndef NAVMAP_ROS__NAVMAP_IO_HPP_
#define NAVMAP_ROS__NAVMAP_IO_HPP_

/**
 * @file navmap_io.hpp
 * @brief Save/Load utilities for NavMap to/from disk using ROS 2 CDR serialization.
 *
 * Two API layers:
 *  1) **Message-level**: save/load `navmap_ros_interfaces::msg::NavMap` directly.
 *  2) **Core-level (optional)**: save/load `navmap::NavMap` if the core headers
 *     are available, using `navmap_ros::to_msg` / `navmap_ros::from_msg`.
 *
 * @par File format
 *  A small binary envelope followed by the ROS 2-serialized payload:
 *  - Magic header
 *  - Version (uint32)
 *  - Payload size (uint64, little-endian)
 *  - Serialized message payload (CDR)
 *
 *  If the magic header is not present, the loader falls back to interpreting the whole file
 *  as a raw CDR payload.
 */

#include <string>
#include <system_error>

#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"

#include "navmap_ros_interfaces/msg/nav_map.hpp"

/**
 * @namespace navmap_ros::io
 * @brief IO helpers to persist NavMap objects to/from disk.
 */
namespace navmap_ros::io
{

/**
 * @brief Save options.
 *
 * @note Currently a placeholder for future extensions (e.g., compression).
 */
struct SaveOptions
{
  /// If @c true, save in binary (CDR). Reserved for future options.
  bool binary{true};
};

/**
 * @brief Save a `NavMap` message to disk using ROS 2 CDR serialization.
 *
 * The resulting file contains a small header (magic, version, size) followed by the CDR payload.
 *
 * @param[in] msg   Message to save.
 * @param[in] path  Destination file path.
 * @param[in] options  Save options (currently unused, reserved).
 * @param[out] ec   Optional error code; if not null, it will be set on failure.
 * @return @c true on success, @c false otherwise (and @p ec is set if provided).
 *
 * @note The format is portable across ROS 2 nodes using the same message type.
 */
bool save_msg_to_file(
  const navmap_ros_interfaces::msg::NavMap & msg,
  const std::string & path,
  const SaveOptions & options = {},
  std::error_code * ec = nullptr);

/**
 * @brief Load a `NavMap` message from a file created by this saver.
 *
 * The loader first tries to parse the header; if the magic header is missing, it falls back
 * to treating the whole file as a raw CDR payload.
 *
 * @param[in]  path     Source file path.
 * @param[out] out_msg  Output message deserialized from file.
 * @param[out] ec       Optional error code; if not null, it will be set on failure.
 * @return @c true on success, @c false otherwise (and @p ec is set if provided).
 */
bool load_msg_from_file(
  const std::string & path,
  navmap_ros_interfaces::msg::NavMap & out_msg,
  std::error_code * ec = nullptr);

/**
 * @brief Save a `navmap::NavMap` by converting it to a message and serializing to disk.
 *
 * Requires `navmap_core` headers and `navmap_ros::to_msg`.
 *
 * @param[in] map    Core map to save.
 * @param[in] path   Destination file path.
 * @param[in] options Save options (currently unused, reserved).
 * @param[out] ec    Optional error code; if not null, it will be set on failure.
 * @return @c true on success, @c false otherwise.
 *
 * @see save_msg_to_file()
 */
bool save_to_file(
  const navmap::NavMap & map,
  const std::string & path,
  const SaveOptions & options = {},
  std::error_code * ec = nullptr);

/**
 * @brief Load a `navmap::NavMap` by deserializing a message and converting to core.
 *
 * Requires `navmap_core` headers and `navmap_ros::from_msg`.
 *
 * @param[in]  path     Source file path.
 * @param[out] out_map  Output core map.
 * @param[out] ec       Optional error code; if not null, it will be set on failure.
 * @return @c true on success, @c false otherwise.
 *
 * @see load_msg_from_file()
 */
bool load_from_file(
  const std::string & path,
  navmap::NavMap & out_map,
  std::error_code * ec = nullptr);

}  // namespace navmap_ros::io

#endif  // NAVMAP_ROS__NAVMAP_IO_HPP_
