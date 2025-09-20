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


#include <fstream>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <cerrno>

#include "navmap_ros/navmap_io.hpp"

#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"


#include <rclcpp/serialized_message.hpp>
#include <rclcpp/serialization.hpp>

namespace navmap_ros::io
{

namespace
{
// Magic header "NAVMAP\0" (7 chars + NUL), version uint32_t = 1, payload_size uint64_t (LE)
static constexpr char kMagic[8] = {'N', 'A', 'V', 'M', 'A', 'P', '\0', '\0'};
static constexpr uint32_t kVersion = 1;

inline void set_error(std::error_code * ec, int ev = EIO)
{
  if (ec) {*ec = std::error_code(ev, std::generic_category());}
}

} // namespace

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

bool load_msg_from_file(
  const std::string & path,
  navmap_ros_interfaces::msg::NavMap & out_msg,
  std::error_code * ec)
{
  using MsgT = navmap_ros_interfaces::msg::NavMap;
  try {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) {set_error(ec, ENOENT); return false;}

    // Read magic
    char magic[8] = {};
    ifs.read(magic, sizeof(magic));
    if (!ifs) {set_error(ec); return false;}

    uint32_t version = 0;
    uint64_t payload_size = 0;
    bool has_header = std::memcmp(magic, kMagic, sizeof(kMagic)) == 0;
    if (has_header) {
      ifs.read(reinterpret_cast<char *>(&version), sizeof(version));
      ifs.read(reinterpret_cast<char *>(&payload_size), sizeof(payload_size));
      if (!ifs || version != kVersion) {set_error(ec); return false;}
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

    // Deserialize ROS 2 CDR
    rclcpp::SerializedMessage serialized(buffer.size());
    auto & rcl_ser = serialized.get_rcl_serialized_message();
    std::memcpy(rcl_ser.buffer, buffer.data(), buffer.size());
    rcl_ser.buffer_length = buffer.size();

    rclcpp::Serialization<MsgT> ser;
    ser.deserialize_message(&serialized, &out_msg);

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

} // namespace navmap_ros::io
