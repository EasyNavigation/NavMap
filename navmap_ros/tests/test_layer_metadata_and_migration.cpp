// Copyright 2026 Intelligent Robotics Lab
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

#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "navmap_core/NavMap.hpp"
#include "navmap_core/Occupancy.hpp"
#include "navmap_ros/conversions.hpp"
#include "navmap_ros/navmap_io.hpp"
#include "navmap_ros_interfaces/msg/nav_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navmap_ros_interfaces/msg/nav_map_v1.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

namespace fs = std::filesystem;
using navmap::NavMap;

namespace
{

NavMap small_map()
{
  NavMap nm;
  const std::size_t s = nm.create_surface("map");
  for (int j = 0; j <= 2; ++j) {
    for (int i = 0; i <= 2; ++i) {
      nm.add_vertex({static_cast<float>(i), static_cast<float>(j), 0.0f});
    }
  }
  auto v = [](int i, int j) {return static_cast<uint32_t>(j * 3 + i);};
  for (int j = 0; j < 2; ++j) {
    for (int i = 0; i < 2; ++i) {
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j), v(i + 1, j + 1)));
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j + 1), v(i, j + 1)));
    }
  }
  nm.rebuild_geometry_accels();
  return nm;
}

// A map with one layer of each type and metadata.
NavMap rich_map()
{
  NavMap nm = small_map();
  nm.add_layer<uint8_t>("occupancy", "occupancy", "", uint8_t{0});
  nm.set_layer_unknown("occupancy", 255.0);
  nm.layer_set<uint8_t>("occupancy", 1, 255);
  nm.add_layer<float>("elevation", "terrain height", "m", 1.5f);
  nm.set_layer_unknown("elevation", std::numeric_limits<double>::quiet_NaN());
  nm.layer_set<float>("elevation", 2, std::numeric_limits<float>::quiet_NaN());
  nm.add_layer<double>("cost", "", "", 0.25);
  navmap::OccupancyLayer occ(nm);
  occ.hit(3);
  occ.miss(4);
  return nm;
}

void expect_rich(const NavMap & nm)
{
  EXPECT_EQ(nm.layer_type_name("occupancy_logodds"), "int16");
  EXPECT_EQ(nm.layer_get<int16_t>("occupancy_logodds", 3), 847);
  EXPECT_EQ(nm.layer_get<int16_t>("occupancy_logodds", 4), -405);
  EXPECT_TRUE(nm.is_unknown("occupancy_logodds", 0));
  EXPECT_TRUE(nm.is_unknown("occupancy", 1));
  EXPECT_FALSE(nm.is_unknown("occupancy", 0));
  EXPECT_TRUE(nm.is_unknown("elevation", 2));
  EXPECT_FLOAT_EQ(nm.layer_get<float>("elevation", 0), 1.5f);
  EXPECT_EQ(nm.get_layer_meta("elevation")->description, "terrain height");
  EXPECT_EQ(nm.get_layer_meta("elevation")->unit, "m");
  EXPECT_FALSE(nm.get_layer_meta("cost")->unknown.has_value());
  EXPECT_DOUBLE_EQ(nm.layer_get<double>("cost", 5), 0.25);
}

std::string tmp_dir(const std::string & name)
{
  const fs::path d = fs::temp_directory_path() / (name + std::to_string(::getpid()));
  fs::remove_all(d);
  fs::create_directories(d);
  return d.string();
}

// Writes `msg` as a version-1 file (the v1 message, with or without the header).
void write_v1(const NavMap & nm, const std::string & path, bool header)
{
  const auto cur = navmap_ros::to_msg(nm);
  navmap_ros_interfaces::msg::NavMapV1 old;
  old.header = cur.header;
  old.positions_x = cur.positions_x;
  old.positions_y = cur.positions_y;
  old.positions_z = cur.positions_z;
  old.navcels_v0 = cur.navcels_v0;
  old.navcels_v1 = cur.navcels_v1;
  old.navcels_v2 = cur.navcels_v2;
  old.surfaces = cur.surfaces;
  for (const auto & l : cur.layers) {
    if (l.type == navmap_ros_interfaces::msg::NavMapLayer::I16) {continue;}  // not in v1
    navmap_ros_interfaces::msg::NavMapLayerV1 o;
    o.name = l.name;
    o.type = l.type;
    o.data_u8 = l.data_u8;
    o.data_f32 = l.data_f32;
    o.data_f64 = l.data_f64;
    old.layers.push_back(o);
  }
  rclcpp::Serialization<navmap_ros_interfaces::msg::NavMapV1> ser;
  rclcpp::SerializedMessage sm;
  ser.serialize_message(&old, &sm);
  const auto & r = sm.get_rcl_serialized_message();
  std::ofstream f(path, std::ios::binary);
  if (header) {
    const char magic[8] = {'N', 'A', 'V', 'M', 'A', 'P', '\0', '\0'};
    const uint32_t ver = 1;
    const uint64_t size = r.buffer_length;
    f.write(magic, 8);
    f.write(reinterpret_cast<const char *>(&ver), 4);
    f.write(reinterpret_cast<const char *>(&size), 8);
  }
  f.write(reinterpret_cast<const char *>(r.buffer), static_cast<std::streamsize>(r.buffer_length));
}

uint32_t file_version(const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  char magic[8];
  uint32_t ver = 0;
  f.read(magic, 8);
  f.read(reinterpret_cast<char *>(&ver), 4);
  return std::string(magic, 6) == "NAVMAP" ? ver : 0;
}

}  // namespace

// ------------------------------------------------------------------ conversions

TEST(LayerMetadata, FullMapRoundTripKeepsInt16AndMetadata)
{
  const NavMap nm = rich_map();
  const auto msg = navmap_ros::to_msg(nm);
  for (const auto & l : msg.layers) {
    if (l.name == "occupancy_logodds") {
      EXPECT_EQ(l.type, navmap_ros_interfaces::msg::NavMapLayer::I16);
      EXPECT_EQ(l.unit, "1/1000 log-odds");
      EXPECT_TRUE(l.has_unknown);
      EXPECT_EQ(l.data_i16.size(), nm.navcels.size());
    }
    if (l.name == "elevation") {
      EXPECT_TRUE(l.has_unknown);
      EXPECT_TRUE(std::isnan(l.unknown_value));
    }
    if (l.name == "cost") {EXPECT_FALSE(l.has_unknown);}
  }
  expect_rich(navmap_ros::from_msg(msg));
}

TEST(LayerMetadata, SingleLayerMessagesCarryMetadataAndRetype)
{
  const NavMap src = rich_map();
  NavMap dst = small_map();
  for (const auto & name : {"occupancy_logodds", "occupancy", "elevation", "cost"}) {
    navmap_ros::from_msg(navmap_ros::to_msg(src, name), dst);
  }
  expect_rich(dst);
  // A layer message of another type replaces the existing layer and its metadata
  NavMap other = small_map();
  other.add_layer<float>("occupancy", "now a float", "%", 0.5f);
  navmap_ros::from_msg(navmap_ros::to_msg(other, "occupancy"), dst);
  EXPECT_EQ(dst.layer_type_name("occupancy"), "float");
  EXPECT_FLOAT_EQ(dst.layer_get<float>("occupancy", 0), 0.5f);
  EXPECT_FALSE(dst.get_layer_meta("occupancy")->unknown.has_value());
  EXPECT_EQ(dst.get_layer_meta("occupancy")->description, "now a float");
}

TEST(LayerMetadata, FileRoundTripIsVersion2)
{
  const std::string dir = tmp_dir("navmap_v2_");
  const std::string path = dir + "/m.navmap";
  ASSERT_TRUE(navmap_ros::io::save_to_file(rich_map(), path));
  EXPECT_EQ(file_version(path), 2u);
  NavMap back;
  ASSERT_TRUE(navmap_ros::io::load_from_file(path, back));
  expect_rich(back);
  fs::remove_all(dir);
}

// ------------------------------------------------------------------ migration of version 1

TEST(FormatMigration, Version1FilesLoadAndAreRewrittenAsVersion2)
{
  for (const bool header : {true, false}) {
    const std::string dir = tmp_dir(header ? "navmap_v1h_" : "navmap_v1n_");
    const std::string path = dir + "/old.navmap";
    write_v1(rich_map(), path, header);
    EXPECT_NE(file_version(path), 2u);
    NavMap nm;
    std::error_code ec;
    ASSERT_TRUE(navmap_ros::io::load_from_file(path, nm, &ec)) << ec.message();
    // Geometry and old layers kept; no metadata existed in version 1
    EXPECT_EQ(nm.navcels.size(), 8u);
    EXPECT_EQ(nm.layer_get<uint8_t>("occupancy", 1), 255);
    EXPECT_FLOAT_EQ(nm.layer_get<float>("elevation", 0), 1.5f);
    EXPECT_FALSE(nm.has_layer("occupancy_logodds"));
    EXPECT_FALSE(nm.is_unknown("occupancy", 1));
    // Rewritten in place, and the next load is a plain version-2 load
    EXPECT_EQ(file_version(path), 2u) << "header " << header;
    EXPECT_FALSE(fs::exists(path + ".migrating"));
    NavMap again;
    ASSERT_TRUE(navmap_ros::io::load_from_file(path, again));
    EXPECT_EQ(again.layer_get<uint8_t>("occupancy", 1), 255);
    fs::remove_all(dir);
  }
}

TEST(FormatMigration, ReadOnlyLocationStillLoadsAndKeepsTheFile)
{
  if (::geteuid() == 0) {GTEST_SKIP() << "root can write anywhere";}
  const std::string dir = tmp_dir("navmap_v1ro_");
  const std::string path = dir + "/old.navmap";
  write_v1(rich_map(), path, true);
  fs::permissions(dir, fs::perms::owner_read | fs::perms::owner_exec);
  NavMap nm;
  EXPECT_TRUE(navmap_ros::io::load_from_file(path, nm));
  EXPECT_EQ(nm.layer_get<uint8_t>("occupancy", 1), 255);
  EXPECT_EQ(file_version(path), 1u);  // could not be rewritten
  fs::permissions(dir, fs::perms::owner_all);
  fs::remove_all(dir);
}

TEST(FormatMigration, NewerOrBrokenFilesAreRejected)
{
  const std::string dir = tmp_dir("navmap_vbad_");
  const std::string path = dir + "/m.navmap";
  ASSERT_TRUE(navmap_ros::io::save_to_file(rich_map(), path));
  {
    std::fstream f(path, std::ios::binary | std::ios::in | std::ios::out);
    f.seekp(8);
    const uint32_t future = 3;
    f.write(reinterpret_cast<const char *>(&future), 4);
  }
  NavMap nm;
  std::error_code ec;
  EXPECT_FALSE(navmap_ros::io::load_from_file(path, nm, &ec));
  EXPECT_TRUE(ec);
  EXPECT_FALSE(navmap_ros::io::load_from_file(dir + "/missing.navmap", nm, &ec));
  fs::remove_all(dir);
}

TEST(LayerMetadata, OccupancyGridUnknownCellsAreDeclaredUnknown)
{
  // As slam_toolbox publishes it: -1 unknown, 0 free, 100 occupied
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = "map";
  g.info.width = 3;
  g.info.height = 1;
  g.info.resolution = 0.5f;
  g.data = {-1, 0, 100};
  const NavMap nm = navmap_ros::from_occupancy_grid(g);
  ASSERT_EQ(nm.navcels.size(), 6u);
  EXPECT_TRUE(nm.is_unknown("occupancy", 0));
  EXPECT_TRUE(nm.is_unknown("occupancy", 1));
  EXPECT_FALSE(nm.is_unknown("occupancy", 2));
  EXPECT_FALSE(nm.is_unknown("occupancy", 4));
  // ...and the declaration travels with the map
  EXPECT_TRUE(navmap_ros::from_msg(navmap_ros::to_msg(nm)).is_unknown("occupancy", 0));
}
