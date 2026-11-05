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

#include <algorithm>
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
#include "zlib.h"

namespace navmap_ros::io
{

namespace
{
// Magic "NAVMAP\0\0", then (little endian):
//  v1: version u32, payload_size u64, CDR payload (also files without header). Migrated on load.
//  v2: version u32, codec u32, raw_size u64, stored_size u64, payload (CDR, compressed by codec).
//      Adds layer metadata (unknown, description, unit), int16 layers, links and geometry ids.
static constexpr char kMagic[8] = {'N', 'A', 'V', 'M', 'A', 'P', '\0', '\0'};
static constexpr uint32_t kVersion = 2;
static constexpr uint32_t kCodecNone = 0;
static constexpr uint32_t kCodecZlib = 1;

inline void set_error(std::error_code * ec, int ev = EIO)
{
  if (ec) {*ec = std::error_code(ev, std::generic_category());}
}

// Each sequence: its elements (with alignment) and a 4-byte length; strings and headers: a
// generous fixed allowance each (kSmall)
std::uint64_t seq(std::size_t n, std::size_t elem)
{
  return static_cast<std::uint64_t>(n) * elem + 4 + 8;
}
constexpr std::uint64_t kSmall = 512;

std::size_t layer_element_size(navmap::LayerType t)
{
  switch (t) {
    case navmap::LayerType::U8: return 1;
    case navmap::LayerType::I16: return 2;
    case navmap::LayerType::F64: return 8;
    default: return 4;   // F32
  }
}

}  // namespace

std::uint64_t estimated_serialized_size(const navmap::NavMap & map)
{
  // The same arrays as to_msg(map) fills, counted without building them
  std::uint64_t n = kSmall;
  n += 3 * seq(map.positions.x.size(), 4);
  if (map.colors.has_value()) {n += 4 * seq(map.positions.x.size(), 1);}
  n += 3 * seq(map.navcels.size(), 4);
  for (const auto & s : map.surfaces) {
    n += kSmall + s.frame_id.size() + seq(s.navcels.size(), 4);
  }
  for (const auto & name : map.layers.list()) {
    if (const auto l = map.layers.get(name)) {
      const auto meta = map.get_layer_meta(name);
      n += kSmall + name.size() + (meta ? meta->description.size() + meta->unit.size() : 0) +
        seq(l->size(), layer_element_size(l->type()));
    }
  }
  n += static_cast<std::uint64_t>(map.links().size()) * kSmall;
  return n;
}

std::uint64_t estimated_serialized_size(const navmap_ros_interfaces::msg::NavMap & msg)
{
  std::uint64_t n = kSmall;   // header, scalars
  n += seq(msg.positions_x.size(), 4) + seq(msg.positions_y.size(), 4) +
    seq(msg.positions_z.size(), 4);
  n += seq(msg.colors_r.size(), 1) + seq(msg.colors_g.size(), 1) + seq(msg.colors_b.size(), 1) +
    seq(msg.colors_a.size(), 1);
  n += seq(msg.navcels_v0.size(), 4) + seq(msg.navcels_v1.size(), 4) +
    seq(msg.navcels_v2.size(), 4);
  for (const auto & s : msg.surfaces) {
    n += kSmall + s.frame_id.size() + seq(s.navcels.size(), 4);
  }
  for (const auto & l : msg.layers) {
    n += kSmall + l.name.size() + l.description.size() + l.unit.size() +
      seq(l.indices.size(), 4) + seq(l.data_u8.size(), 1) + seq(l.data_f32.size(), 4) +
      seq(l.data_f64.size(), 8) + seq(l.data_i16.size(), 2);
  }
  n += static_cast<std::uint64_t>(msg.links.size()) * kSmall;
  return n;
}

namespace
{

// false (and the error set) if \p estimate bytes are more than the saver attempts
bool fits_in_a_message(
  std::uint64_t estimate, const std::string & path, const SaveOptions & options,
  std::error_code * ec)
{
  if (estimate <= options.max_serialized_bytes) {return true;}
  RCLCPP_ERROR(
    rclcpp::get_logger("navmap_io"),
    "cannot save %s: the map serialises to about %.2f GB, more than the %.2f GB a ROS 2 (CDR) "
    "message holds; save it in pieces or coarser", path.c_str(), estimate / 1e9,
    options.max_serialized_bytes / 1e9);
  set_error(ec, EFBIG);
  return false;
}

bool fits_in_a_message(
  const navmap::NavMap & map, const std::string & path, const SaveOptions & options,
  std::error_code * ec)
{
  return fits_in_a_message(estimated_serialized_size(map), path, options, ec);
}

}  // namespace

bool save_msg_to_file(
  const navmap_ros_interfaces::msg::NavMap & msg,
  const std::string & path,
  const SaveOptions & options,
  std::error_code * ec)
{
  using MsgT = navmap_ros_interfaces::msg::NavMap;
  // Beyond what the CDR serialiser holds: refused before allocating gigabytes, with the cause
  // (it threw a non-std exception from deep inside, reported as a bare I/O error)
  if (!fits_in_a_message(estimated_serialized_size(msg), path, options, ec)) {return false;}
  try {
    rclcpp::SerializedMessage serialized;
    rclcpp::Serialization<MsgT> ser;
    ser.serialize_message(&msg, &serialized);
    const auto & rcl_ser = serialized.get_rcl_serialized_message();
    const auto * raw = static_cast<const uint8_t *>(rcl_ser.buffer);
    const uint64_t raw_size = rcl_ser.buffer_length;

    uint32_t codec = kCodecNone;
    const uint8_t * payload = raw;
    uint64_t stored_size = raw_size;
    std::vector<uint8_t> packed;
    if (options.compression == Compression::ZLIB) {
      uLongf n = compressBound(static_cast<uLong>(raw_size));
      packed.resize(n);
      const int level = std::clamp(options.level, 1, 9);
      if (compress2(packed.data(), &n, raw, static_cast<uLong>(raw_size), level) != Z_OK) {
        set_error(ec); return false;
      }
      codec = kCodecZlib;
      payload = packed.data();
      stored_size = n;
    }

    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) {set_error(ec, EACCES); return false;}
    ofs.write(kMagic, sizeof(kMagic));
    const uint32_t ver = kVersion;
    ofs.write(reinterpret_cast<const char *>(&ver), sizeof(ver));
    ofs.write(reinterpret_cast<const char *>(&codec), sizeof(codec));
    ofs.write(reinterpret_cast<const char *>(&raw_size), sizeof(raw_size));
    ofs.write(reinterpret_cast<const char *>(&stored_size), sizeof(stored_size));
    ofs.write(reinterpret_cast<const char *>(payload), static_cast<std::streamsize>(stored_size));
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
    uint32_t codec = kCodecNone;
    uint64_t raw_size = 0, stored_size = 0;
    if (std::memcmp(magic, kMagic, sizeof(kMagic)) == 0) {
      ifs.read(reinterpret_cast<char *>(&version), sizeof(version));
      if (!ifs || version < 1 || version > kVersion) {set_error(ec); return false;}
      if (version >= 2) {
        ifs.read(reinterpret_cast<char *>(&codec), sizeof(codec));
        ifs.read(reinterpret_cast<char *>(&raw_size), sizeof(raw_size));
      }
      ifs.read(reinterpret_cast<char *>(&stored_size), sizeof(stored_size));
      if (!ifs || (codec != kCodecNone && codec != kCodecZlib)) {set_error(ec); return false;}
      if (version == 1) {raw_size = stored_size;}
    } else {
      ifs.clear();
      ifs.seekg(0, std::ios::end);
      auto end = ifs.tellg();
      if (end <= 0) {set_error(ec); return false;}
      stored_size = raw_size = static_cast<uint64_t>(end);
      ifs.seekg(0, std::ios::beg);
    }
    // A truncated or corrupt header must not allocate blindly
    const auto here = ifs.tellg();
    ifs.seekg(0, std::ios::end);
    const auto left = static_cast<uint64_t>(ifs.tellg() - here);
    ifs.seekg(here);
    if (stored_size > left || (codec == kCodecNone && raw_size != stored_size)) {
      set_error(ec); return false;
    }

    std::vector<uint8_t> buffer(static_cast<size_t>(stored_size));
    ifs.read(reinterpret_cast<char *>(buffer.data()), static_cast<std::streamsize>(stored_size));
    if (!ifs) {set_error(ec); return false;}
    ifs.close();
    if (codec == kCodecZlib) {
      std::vector<uint8_t> raw(static_cast<size_t>(raw_size));
      uLongf n = static_cast<uLongf>(raw_size);
      if (uncompress(raw.data(), &n, buffer.data(), static_cast<uLong>(stored_size)) != Z_OK ||
        n != raw_size)
      {
        set_error(ec); return false;
      }
      buffer.swap(raw);
    }

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
  // Refused before to_msg builds the message (gigabytes more than the map itself)
  if (!fits_in_a_message(map, path, options, ec)) {return false;}
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
