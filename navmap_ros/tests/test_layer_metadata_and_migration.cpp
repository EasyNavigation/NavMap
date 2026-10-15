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
using navmap::NavCelId;
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

// ------------------------------------------------------------------ compression

// A map large enough for compression to matter: a 60 x 60 grid with a few layers
NavMap grid_map()
{
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = "map";
  g.info.resolution = 0.1f;
  g.info.width = 60;
  g.info.height = 60;
  g.data.assign(60 * 60, 0);
  for (int i = 0; i < 60; ++i) {g.data[30 * 60 + i] = 100; g.data[i] = -1;}
  NavMap nm = navmap_ros::from_occupancy_grid(g);
  nm.add_layer<float>("elevation", "", "m", 0.25f);
  return nm;
}

struct Envelope
{
  uint32_t version{0}, codec{0};
  uint64_t raw{0}, stored{0}, file{0};
};

Envelope envelope(const std::string & path)
{
  Envelope e;
  std::ifstream f(path, std::ios::binary);
  f.seekg(8);
  f.read(reinterpret_cast<char *>(&e.version), 4);
  f.read(reinterpret_cast<char *>(&e.codec), 4);
  f.read(reinterpret_cast<char *>(&e.raw), 8);
  f.read(reinterpret_cast<char *>(&e.stored), 8);
  e.file = fs::file_size(path);
  return e;
}

void patch(const std::string & path, std::streamoff at, const void * bytes, std::size_t n)
{
  std::fstream f(path, std::ios::binary | std::ios::in | std::ios::out);
  f.seekp(at);
  f.write(static_cast<const char *>(bytes), static_cast<std::streamsize>(n));
}

void expect_same(const NavMap & a, const NavMap & b)
{
  EXPECT_EQ(a.positions.x, b.positions.x);
  EXPECT_EQ(a.positions.z, b.positions.z);
  ASSERT_EQ(a.navcels.size(), b.navcels.size());
  for (navmap::NavCelId c = 0; c < a.navcels.size(); ++c) {
    ASSERT_EQ(a.layer_get<uint8_t>("occupancy", c), b.layer_get<uint8_t>("occupancy", c));
    ASSERT_EQ(a.layer_get<float>("elevation", c), b.layer_get<float>("elevation", c));
  }
  EXPECT_TRUE(b.get_layer_meta("occupancy")->unknown.has_value());
  EXPECT_EQ(a.geometry_id(), b.geometry_id());
}

TEST(Compression, ZlibByDefaultSmallerAndIdenticalOnLoad)
{
  const std::string dir = tmp_dir("navmap_z_");
  const NavMap nm = grid_map();
  navmap_ros::io::SaveOptions raw_opt;
  raw_opt.compression = navmap_ros::io::Compression::NONE;
  ASSERT_TRUE(navmap_ros::io::save_to_file(nm, dir + "/z.navmap"));
  ASSERT_TRUE(navmap_ros::io::save_to_file(nm, dir + "/raw.navmap", raw_opt));
  const Envelope z = envelope(dir + "/z.navmap"), r = envelope(dir + "/raw.navmap");
  EXPECT_EQ(z.version, 2u);
  EXPECT_EQ(z.codec, 1u);
  EXPECT_EQ(r.codec, 0u);
  EXPECT_EQ(z.raw, r.raw);                 // same CDR payload
  EXPECT_EQ(r.stored, r.raw);
  EXPECT_EQ(z.file, 32 + z.stored);        // header + payload, nothing else
  EXPECT_LT(z.stored, r.raw / 3);          // a mostly uniform grid compresses well
  for (const auto * f : {"/z.navmap", "/raw.navmap"}) {
    NavMap back;
    ASSERT_TRUE(navmap_ros::io::load_from_file(dir + f, back)) << f;
    expect_same(nm, back);
  }
  fs::remove_all(dir);
}

TEST(Compression, EveryLevelLoadsAndOutOfRangeLevelsAreClamped)
{
  const std::string dir = tmp_dir("navmap_zl_");
  const NavMap nm = grid_map();
  // The zlib stream header records the level class: 0x01 fastest, 0x9C default, 0xDA best
  auto flevel = [&](int level) {
      std::ifstream f(dir + "/l" + std::to_string(level) + ".navmap", std::ios::binary);
      f.seekg(33);
      return static_cast<unsigned>(static_cast<unsigned char>(f.get()));
    };
  for (const int level : {-5, 0, 1, 6, 9, 42}) {
    navmap_ros::io::SaveOptions o;
    o.level = level;
    const std::string path = dir + "/l" + std::to_string(level) + ".navmap";
    ASSERT_TRUE(navmap_ros::io::save_to_file(nm, path, o)) << level;
    NavMap back;
    ASSERT_TRUE(navmap_ros::io::load_from_file(path, back)) << level;
    expect_same(nm, back);
  }
  // The level is applied, and clamped to [1, 9]
  EXPECT_EQ(flevel(1), 0x01u);
  EXPECT_EQ(flevel(6), 0x9Cu);
  EXPECT_EQ(flevel(9), 0xDAu);
  EXPECT_EQ(flevel(-5), 0x01u);
  EXPECT_EQ(flevel(0), 0x01u);
  EXPECT_EQ(flevel(42), 0xDAu);
  fs::remove_all(dir);
}

TEST(Compression, EmptyMapRoundTrips)
{
  const std::string dir = tmp_dir("navmap_ze_");
  ASSERT_TRUE(navmap_ros::io::save_to_file(NavMap{}, dir + "/e.navmap"));
  NavMap back;
  ASSERT_TRUE(navmap_ros::io::load_from_file(dir + "/e.navmap", back));
  EXPECT_TRUE(back.navcels.empty());
  fs::remove_all(dir);
}

TEST(Compression, CorruptOrTruncatedFilesAreRejectedWithoutCrashing)
{
  const std::string dir = tmp_dir("navmap_zc_");
  const std::string good = dir + "/good.navmap";
  ASSERT_TRUE(navmap_ros::io::save_to_file(grid_map(), good));
  const Envelope e = envelope(good);
  auto fails = [&](const std::string & what, auto && damage) {
      const std::string path = dir + "/bad.navmap";
      fs::copy_file(good, path, fs::copy_options::overwrite_existing);
      damage(path);
      NavMap nm;
      std::error_code ec;
      EXPECT_FALSE(navmap_ros::io::load_from_file(path, nm, &ec)) << what;
      EXPECT_TRUE(ec) << what;
    };
  fails("unknown codec", [](const std::string & p) {const uint32_t c = 7; patch(p, 12, &c, 4);});
  fails("raw size too small", [&](const std::string & p) {
      const uint64_t v = e.raw / 2; patch(p, 16, &v, 8);});
  fails("raw size too large", [&](const std::string & p) {
      const uint64_t v = e.raw + 1; patch(p, 16, &v, 8);});
  fails("stored size beyond the file", [](const std::string & p) {
      const uint64_t v = uint64_t{1} << 40; patch(p, 24, &v, 8);});
  fails("garbage payload", [](const std::string & p) {
      const char junk[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
      patch(p, 32, junk, sizeof(junk));});
  fails("truncated payload", [&](const std::string & p) {fs::resize_file(p, 32 + e.stored / 2);});
  fails("truncated header", [](const std::string & p) {fs::resize_file(p, 20);});
  fails("uncompressed with mismatched sizes", [&](const std::string & p) {
      const uint32_t c = 0; patch(p, 12, &c, 4);});
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

TEST(Links, TravelInMessagesAndFilesAndInvalidOnesAreDropped)
{
  NavMap nm = small_map();
  nm.add_link(0, 7, navmap::LinkType::ELEVATOR, 12.0f, false);
  nm.add_link(2, 5, navmap::LinkType::STAIRS);
  auto msg = navmap_ros::to_msg(nm);
  ASSERT_EQ(msg.links.size(), 2u);
  EXPECT_EQ(msg.links[0].type, navmap_ros_interfaces::msg::NavMapLink::ELEVATOR);
  EXPECT_FALSE(msg.links[0].bidirectional);
  navmap_ros_interfaces::msg::NavMapLink bad;
  bad.from_navcel = 0;
  bad.to_navcel = 999;  // out of range
  msg.links.push_back(bad);
  const NavMap back = navmap_ros::from_msg(msg);
  ASSERT_EQ(back.links().size(), 2u);
  EXPECT_FLOAT_EQ(back.links()[0].length, 12.0f);
  EXPECT_EQ(back.links()[1].type, navmap::LinkType::STAIRS);
  // And in a file
  const std::string dir = tmp_dir("navmap_links_");
  ASSERT_TRUE(navmap_ros::io::save_to_file(nm, dir + "/m.navmap"));
  NavMap loaded;
  ASSERT_TRUE(navmap_ros::io::load_from_file(dir + "/m.navmap", loaded));
  ASSERT_EQ(loaded.links().size(), 2u);
  EXPECT_EQ(loaded.links()[0].to, 7u);
  fs::remove_all(dir);
  // Version-1 files have no links
  write_v1(nm, dir + "_v1.navmap", true);
  NavMap old;
  ASSERT_TRUE(navmap_ros::io::load_from_file(dir + "_v1.navmap", old));
  EXPECT_TRUE(old.links().empty());
  fs::remove(dir + "_v1.navmap");
}

TEST(LayerUpdates, SparseWhenFewCellsChangedAndAppliedOnTheSameGeometry)
{
  NavMap src = small_map();
  auto v = src.add_layer<float>("cost", "c", "", 0.0f);
  NavMap dst = small_map();
  navmap_ros::from_msg(navmap_ros::to_msg(src, "cost"), dst);
  // A new layer: full update
  auto m = navmap_ros::to_layer_update_msg(src, "cost");
  EXPECT_TRUE(m.indices.empty());
  EXPECT_EQ(m.data_f32.size(), src.navcels.size());
  EXPECT_EQ(m.geometry_id, src.geometry_id());
  v->clear_changes();
  // Nothing changed: full (an empty sparse update would mean the whole layer)
  EXPECT_TRUE(navmap_ros::to_layer_update_msg(src, "cost").indices.empty());
  // One cell changed: sparse
  src.layer_set<float>("cost", 5, 2.5f);
  m = navmap_ros::to_layer_update_msg(src, "cost");
  ASSERT_EQ(m.indices, (std::vector<uint32_t>{5}));
  ASSERT_EQ(m.data_f32, (std::vector<float>{2.5f}));
  dst.layers.get("cost")->clear_changes();  // the full load above marked the whole layer
  navmap_ros::from_msg(m, dst);
  EXPECT_FLOAT_EQ(dst.layer_get<float>("cost", 5), 2.5f);
  EXPECT_FLOAT_EQ(dst.layer_get<float>("cost", 4), 0.0f);
  EXPECT_EQ(dst.layers.get("cost")->changed(), (std::vector<NavCelId>{5}));
  // More than the sparse fraction changed: full
  for (NavCelId c = 0; c < 4; ++c) {
    src.layer_set<float>("cost", c, 1.0f);
                                                                          }
  EXPECT_TRUE(navmap_ros::to_layer_update_msg(src, "cost", std_msgs::msg::Header(), 0.25f).indices.empty());
  EXPECT_FALSE(navmap_ros::to_layer_update_msg(src, "cost", std_msgs::msg::Header(), 0.9f).indices.empty());
}

TEST(LayerUpdates, UpdatesForAnotherGeometryOrMalformedAreRejected)
{
  NavMap src = small_map();
  src.add_layer<uint8_t>("occ", "", "", uint8_t{0});
  src.layers.get("occ")->clear_changes();
  src.layer_set<uint8_t>("occ", 1, 254);
  const auto sparse = navmap_ros::to_layer_update_msg(src, "occ");
  ASSERT_FALSE(sparse.indices.empty());
  // Another geometry
  NavMap other = small_map();
  other.add_vertex({5.0f, 5.0f, 0.0f});
  other.add_layer<uint8_t>("occ", "", "", uint8_t{0});
  EXPECT_THROW(navmap_ros::from_msg(sparse, other), std::runtime_error);
  EXPECT_THROW(navmap_ros::from_msg(navmap_ros::to_msg(src, "occ"), other), std::runtime_error);
  // geometry_id 0 (an older publisher): accepted
  auto anon = navmap_ros::to_msg(src, "occ");
  anon.geometry_id = 0;
  EXPECT_NO_THROW(navmap_ros::from_msg(anon, other));
  // Sparse update of a missing layer, or of another type, or with mismatched sizes
  NavMap same = small_map();
  EXPECT_THROW(navmap_ros::from_msg(sparse, same), std::runtime_error);
  same.add_layer<float>("occ", "", "", 0.0f);
  EXPECT_THROW(navmap_ros::from_msg(sparse, same), std::runtime_error);
  NavMap good = small_map();
  good.add_layer<uint8_t>("occ", "", "", uint8_t{0});
  auto bad = sparse;
  bad.data_u8.push_back(7);
  EXPECT_THROW(navmap_ros::from_msg(bad, good), std::runtime_error);
  EXPECT_NO_THROW(navmap_ros::from_msg(sparse, good));
  EXPECT_EQ(good.layer_get<uint8_t>("occ", 1), 254);
  // The full map message carries the geometry id too
  EXPECT_EQ(navmap_ros::to_msg(src).geometry_id, src.geometry_id());
}
