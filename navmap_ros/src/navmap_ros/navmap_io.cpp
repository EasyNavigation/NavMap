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


#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <vector>
#include <cstring>
#include <cerrno>

#include "navmap_ros/navmap_io.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/logging.hpp"
#include "navmap_ros_interfaces/msg/nav_map_v1.hpp"

namespace navmap_ros::io
{

namespace
{
// Magic header "NAVMAP\0" (7 chars + NUL), version uint32_t, payload_size uint64_t (LE).
// Version 1 (also files without header) is migrated on load.
static constexpr char kMagic[8] = {'N', 'A', 'V', 'M', 'A', 'P', '\0', '\0'};
// 2: layer metadata (unknown, description, unit), int16 layers and links
static constexpr uint32_t kVersion = 2;

inline void set_error(std::error_code * ec, int ev = EIO)
{
  if (ec) {*ec = std::error_code(ev, std::generic_category());}
}

}  // namespace

bool save_msg_to_file(
  const navmap_ros_interfaces::msg::NavMap & msg,
  const std::string & path,
  const SaveOptions & /*options*/,
  std::error_code * ec)
{
  using MsgT = navmap_ros_interfaces::msg::NavMap;
  try {
    // Serialize to ROS 2 CDR
    rclcpp::SerializedMessage serialized;
    rclcpp::Serialization<MsgT> ser;
    ser.serialize_message(&msg, &serialized);

    const auto & rcl_ser = serialized.get_rcl_serialized_message();
    const auto payload = static_cast<const uint8_t *>(rcl_ser.buffer);
    const size_t payload_size = rcl_ser.buffer_length;

    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) {set_error(ec, EACCES); return false;}

    // Write header
    ofs.write(reinterpret_cast<const char *>(kMagic), sizeof(kMagic));
    uint32_t ver = kVersion;
    ofs.write(reinterpret_cast<const char *>(&ver), sizeof(ver));
    uint64_t size64 = static_cast<uint64_t>(payload_size);
    ofs.write(reinterpret_cast<const char *>(&size64), sizeof(size64));

    // Write payload
    ofs.write(reinterpret_cast<const char *>(payload), payload_size);
    ofs.flush();
    if (!ofs) {set_error(ec); return false;}
    if (ec) {*ec = {};}
    return true;
  } catch (...) {
    set_error(ec);
    return false;
  }
}

namespace
{

// Version 1 (no layer metadata, no int16 layers) -> current message.
navmap_ros_interfaces::msg::NavMap migrate_v1(const navmap_ros_interfaces::msg::NavMapV1 & old)
{
  navmap_ros_interfaces::msg::NavMap m;
  m.header = old.header;
  m.positions_x = old.positions_x;
  m.positions_y = old.positions_y;
  m.positions_z = old.positions_z;
  m.has_vertex_rgba = old.has_vertex_rgba;
  m.colors_r = old.colors_r;
  m.colors_g = old.colors_g;
  m.colors_b = old.colors_b;
  m.colors_a = old.colors_a;
  m.navcels_v0 = old.navcels_v0;
  m.navcels_v1 = old.navcels_v1;
  m.navcels_v2 = old.navcels_v2;
  m.surfaces = old.surfaces;
  for (const auto & l : old.layers) {
    navmap_ros_interfaces::msg::NavMapLayer n;
    n.header = l.header;
    n.name = l.name;
    n.type = l.type;
    n.data_u8 = l.data_u8;
    n.data_f32 = l.data_f32;
    n.data_f64 = l.data_f64;
    m.layers.push_back(std::move(n));
  }
  return m;
}

template<typename MsgT>
void deserialize(const std::vector<uint8_t> & buffer, MsgT & out)
{
  rclcpp::SerializedMessage serialized(buffer.size());
  auto & rcl_ser = serialized.get_rcl_serialized_message();
  std::memcpy(rcl_ser.buffer, buffer.data(), buffer.size());
  rcl_ser.buffer_length = buffer.size();
  rclcpp::Serialization<MsgT> ser;
  ser.deserialize_message(&serialized, &out);
}

}  // namespace

bool load_msg_from_file(
  const std::string & path,
  navmap_ros_interfaces::msg::NavMap & out_msg,
  std::error_code * ec)
{
  try {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) {set_error(ec, ENOENT); return false;}

    // Read magic
    char magic[8] = {};
    ifs.read(magic, sizeof(magic));
    if (!ifs) {set_error(ec); return false;}

    // Files without header are the oldest format: version 1
    uint32_t version = 1;
    uint64_t payload_size = 0;
    bool has_header = std::memcmp(magic, kMagic, sizeof(kMagic)) == 0;
    if (has_header) {
      ifs.read(reinterpret_cast<char *>(&version), sizeof(version));
      ifs.read(reinterpret_cast<char *>(&payload_size), sizeof(payload_size));
      if (!ifs || version < 1 || version > kVersion) {set_error(ec); return false;}
    } else {
      // No header: treat whole file as payload
      ifs.clear();
      ifs.seekg(0, std::ios::end);
      auto end = ifs.tellg();
      if (end <= 0) {set_error(ec); return false;}
      payload_size = static_cast<uint64_t>(end);
      ifs.seekg(0, std::ios::beg);
    }

    // Read payload
    std::vector<uint8_t> buffer;
    buffer.resize(static_cast<size_t>(payload_size));
    ifs.read(reinterpret_cast<char *>(buffer.data()), static_cast<std::streamsize>(payload_size));
    if (!ifs) {set_error(ec); return false;}
    ifs.close();

    if (version == kVersion) {
      deserialize(buffer, out_msg);
    } else {
      // Deprecated format: migrate, and replace the file with the current format
      navmap_ros_interfaces::msg::NavMapV1 old;
      deserialize(buffer, old);
      out_msg = migrate_v1(old);
      auto logger = rclcpp::get_logger("navmap_io");
      std::error_code wec;
      if (save_msg_to_file(out_msg, path + ".migrating", {}, &wec) &&
        std::rename((path + ".migrating").c_str(), path.c_str()) == 0)
      {
        RCLCPP_WARN(
          logger, "%s: .navmap format version %u is deprecated; migrated to version %u and "
          "rewritten in place", path.c_str(), version, kVersion);
      } else {
        std::remove((path + ".migrating").c_str());
        RCLCPP_WARN(
          logger, "%s: .navmap format version %u is deprecated; loaded through a migration to "
          "version %u, but the file could not be rewritten (save it again to update it)",
          path.c_str(), version, kVersion);
      }
    }

    if (ec) {*ec = {};}
    return true;
  } catch (...) {
    set_error(ec);
    return false;
  }
}

bool save_to_file(
  const navmap::NavMap & map,
  const std::string & path,
  const SaveOptions & options,
  std::error_code * ec)
{
  auto msg = navmap_ros::to_msg(map);
  return save_msg_to_file(msg, path, options, ec);
}

bool load_from_file(
  const std::string & path,
  navmap::NavMap & out_map,
  std::error_code * ec)
{
  navmap_ros_interfaces::msg::NavMap msg;
  if (!load_msg_from_file(path, msg, ec)) {return false;}
  out_map = navmap_ros::from_msg(msg);
  if (ec) {*ec = {};}
  return true;
}

}  // namespace navmap_ros::io
