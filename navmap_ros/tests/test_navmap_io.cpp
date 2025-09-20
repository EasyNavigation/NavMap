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

#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <filesystem>
#include <fstream>
#include <vector>
#include <string>
#include <unistd.h>
#include <unordered_map>

#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"
#include "navmap_ros/navmap_io.hpp"
#include "navmap_ros_interfaces/msg/nav_map.hpp"

namespace
{

std::vector<uint8_t> serializeToBytes(const navmap_ros_interfaces::msg::NavMap & msg)
{
  rclcpp::Serialization<navmap_ros_interfaces::msg::NavMap> ser;
  rclcpp::SerializedMessage serialized;
  ser.serialize_message(&msg, &serialized);
  const auto & rcl_ser = serialized.get_rcl_serialized_message();
  const uint8_t * data = static_cast<const uint8_t *>(rcl_ser.buffer);
  return std::vector<uint8_t>(data, data + rcl_ser.buffer_length);
}

std::string tmp_file(const std::string & name)
{
  namespace fs = std::filesystem;
  fs::path p = fs::temp_directory_path() / (name + std::to_string(::getpid()) + ".navmap");
  return p.string();
}

void fill_basic_header(navmap_ros_interfaces::msg::NavMap & msg, const std::string & frame = "map")
{
  msg.header.frame_id = frame;
  // leave stamp default
}

} // namespace

// --- helpers: semantic comparison for messages ---

template<typename T>
static void ExpectVecEq(const std::vector<T> & a, const std::vector<T> & b, const char * what)
{
  ASSERT_EQ(a.size(), b.size()) << what << " size mismatch";
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i], b[i]) << what << " differs at " << i;
  }
}

static void ExpectVecFloatEq(
  const std::vector<float> & a, const std::vector<float> & b,
  const char * what)
{
  ASSERT_EQ(a.size(), b.size()) << what << " size mismatch";
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_FLOAT_EQ(a[i], b[i]) << what << " differs at " << i;
  }
}

static void ExpectVecNear64(
  const std::vector<double> & a, const std::vector<double> & b,
  const char * what, double eps = 1e-12)
{
  ASSERT_EQ(a.size(), b.size()) << what << " size mismatch";
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_NEAR(a[i], b[i], eps) << what << " differs at " << i;
  }
}

static void ExpectNavMapMsgEqualSemantic(
  const navmap_ros_interfaces::msg::NavMap & A,
  const navmap_ros_interfaces::msg::NavMap & B)
{
  // Header: frame must match; stamp puede variar → lo ignoramos
  EXPECT_EQ(A.header.frame_id, B.header.frame_id);

  // Geometry
  ExpectVecFloatEq(A.positions_x, B.positions_x, "positions_x");
  ExpectVecFloatEq(A.positions_y, B.positions_y, "positions_y");
  ExpectVecFloatEq(A.positions_z, B.positions_z, "positions_z");

  ExpectVecEq(A.navcels_v0, B.navcels_v0, "navcels_v0");
  ExpectVecEq(A.navcels_v1, B.navcels_v1, "navcels_v1");
  ExpectVecEq(A.navcels_v2, B.navcels_v2, "navcels_v2");

  // Vertex RGBA
  EXPECT_EQ(A.has_vertex_rgba, B.has_vertex_rgba);
  if (A.has_vertex_rgba || B.has_vertex_rgba) {
    ExpectVecEq(A.colors_r, B.colors_r, "colors_r");
    ExpectVecEq(A.colors_g, B.colors_g, "colors_g");
    ExpectVecEq(A.colors_b, B.colors_b, "colors_b");
    ExpectVecEq(A.colors_a, B.colors_a, "colors_a");
  }

  ASSERT_EQ(A.surfaces.size(), B.surfaces.size()) << "surfaces size mismatch";
  for (size_t i = 0; i < A.surfaces.size(); ++i) {
    EXPECT_EQ(A.surfaces[i].frame_id, B.surfaces[i].frame_id);
    EXPECT_EQ(A.surfaces[i].navcels.size(), B.surfaces[i].navcels.size());
  }

  ASSERT_EQ(A.layers.size(), B.layers.size()) << "layers count mismatch";
  std::unordered_map<std::string, const navmap_ros_interfaces::msg::NavMapLayer *> mapA, mapB;
  for (const auto & L : A.layers) {
    mapA[L.name] = &L;
  }
  for (const auto & L : B.layers) {
    mapB[L.name] = &L;
  }

  for (const auto & [name, La] : mapA) {
    auto it = mapB.find(name);
    ASSERT_NE(it, mapB.end()) << "layer missing in B: " << name;
    const auto * Lb = it->second;
    EXPECT_EQ(La->type, Lb->type) << "layer type mismatch for " << name;

    switch (La->type) {
      case 0: // u8
        ExpectVecEq(La->data_u8, Lb->data_u8, (name + ".data_u8").c_str());
        break;
      case 1: // f32
        ExpectVecFloatEq(La->data_f32, Lb->data_f32, (name + ".data_f32").c_str());
        break;
      case 2: // f64
        ExpectVecNear64(La->data_f64, Lb->data_f64, (name + ".data_f64").c_str());
        break;
      default:
        FAIL() << "unknown layer type for " << name;
    }
  }
}

TEST(NavMapIoMsg, EmptyMinimal)
{
  navmap_ros_interfaces::msg::NavMap msg;
  fill_basic_header(msg);

  auto before = serializeToBytes(msg);
  std::string path = tmp_file("empty_");

  std::error_code ec;
  ASSERT_TRUE(navmap_ros::io::save_msg_to_file(msg, path, {}, &ec)) << ec.message();
  navmap_ros_interfaces::msg::NavMap loaded;
  ASSERT_TRUE(navmap_ros::io::load_msg_from_file(path, loaded, &ec)) << ec.message();
  auto after = serializeToBytes(loaded);

  EXPECT_EQ(before, after);
  std::filesystem::remove(path);
}

TEST(NavMapIoMsg, SingleTriangleWithU8AndVertexRGBA)
{
  navmap_ros_interfaces::msg::NavMap msg;
  fill_basic_header(msg);

  // 3 vertices
  msg.positions_x = {0.0f, 1.0f, 0.0f};
  msg.positions_y = {0.0f, 0.0f, 1.0f};
  msg.positions_z = {0.0f, 0.0f, 0.0f};

  // 1 triangle
  msg.navcels_v0 = {0};
  msg.navcels_v1 = {1};
  msg.navcels_v2 = {2};

  // Vertex colors
  msg.has_vertex_rgba = true;
  msg.colors_r = {255, 0, 0};
  msg.colors_g = {0, 255, 0};
  msg.colors_b = {0, 0, 255};
  msg.colors_a = {255, 255, 255};

  // U8 layer with one value
  navmap_ros_interfaces::msg::NavMapLayer layer_u8;
  layer_u8.name = "occupancy_like";
  layer_u8.type = 0; // u8
  layer_u8.data_u8 = {42};
  msg.layers.push_back(layer_u8);

  auto before = serializeToBytes(msg);
  std::string path = tmp_file("tri_u8_rgba_");

  std::error_code ec;
  ASSERT_TRUE(navmap_ros::io::save_msg_to_file(msg, path, {}, &ec)) << ec.message();
  navmap_ros_interfaces::msg::NavMap loaded;
  ASSERT_TRUE(navmap_ros::io::load_msg_from_file(path, loaded, &ec)) << ec.message();
  auto after = serializeToBytes(loaded);

  EXPECT_EQ(before, after);
  std::filesystem::remove(path);
}

TEST(NavMapIoMsg, MultiTriangles_MultiLayers)
{
  navmap_ros_interfaces::msg::NavMap msg;
  fill_basic_header(msg, "odom");

  // 4 vertices -> 2 triangles (0,1,2) and (1,3,2)
  msg.positions_x = {0.0f, 1.0f, 0.0f, 1.0f};
  msg.positions_y = {0.0f, 0.0f, 1.0f, 1.0f};
  msg.positions_z = {0.0f, 0.1f, 0.0f, 0.2f};

  msg.navcels_v0 = {0, 1};
  msg.navcels_v1 = {1, 3};
  msg.navcels_v2 = {2, 2};

  // Optional vertex RGBA (size = #vertices)
  msg.has_vertex_rgba = true;
  msg.colors_r = {10, 20, 30, 40};
  msg.colors_g = {50, 60, 70, 80};
  msg.colors_b = {90, 100, 110, 120};
  msg.colors_a = {255, 200, 150, 100};

  // Layers
  navmap_ros_interfaces::msg::NavMapLayer layer_u8;
  layer_u8.name = "u8";
  layer_u8.type = 0;
  layer_u8.data_u8 = {1, 253};

  navmap_ros_interfaces::msg::NavMapLayer layer_f32;
  layer_f32.name = "f32";
  layer_f32.type = 1;
  layer_f32.data_f32 = {0.0f, 3.14f};

  navmap_ros_interfaces::msg::NavMapLayer layer_f64;
  layer_f64.name = "f64";
  layer_f64.type = 2;
  layer_f64.data_f64 = {1.0, 2.0};

  msg.layers = {layer_u8, layer_f32, layer_f64};

  auto before = serializeToBytes(msg);
  std::string path = tmp_file("multi_layers_");

  std::error_code ec;
  ASSERT_TRUE(navmap_ros::io::save_msg_to_file(msg, path, {}, &ec)) << ec.message();
  navmap_ros_interfaces::msg::NavMap loaded;
  ASSERT_TRUE(navmap_ros::io::load_msg_from_file(path, loaded, &ec)) << ec.message();
  auto after = serializeToBytes(loaded);

  EXPECT_EQ(before, after);
  std::filesystem::remove(path);
}


TEST(NavMapIoCore, RoundtripViaCoreAndMsgCompare)
{
  using navmap_ros_interfaces::msg::NavMap;

  NavMap msg;
  msg.header.frame_id = "base_link";

  msg.positions_x = {0.0f, 2.0f, 0.0f, 2.0f, 1.0f};
  msg.positions_y = {0.0f, 0.0f, 2.0f, 2.0f, 1.0f};
  msg.positions_z = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  msg.navcels_v0 = {0, 1, 4};
  msg.navcels_v1 = {1, 3, 2};
  msg.navcels_v2 = {2, 2, 0};

  msg.has_vertex_rgba = true;
  msg.colors_r = {255, 0, 0, 255, 128};
  msg.colors_g = {0, 255, 0, 255, 128};
  msg.colors_b = {0, 0, 255, 255, 128};
  msg.colors_a = {255, 255, 255, 255, 200};

  navmap_ros_interfaces::msg::NavMapLayer u8;
  u8.name = "occ"; u8.type = 0; u8.data_u8 = {0, 254, 42};
  navmap_ros_interfaces::msg::NavMapLayer f32;
  f32.name = "cost"; f32.type = 1; f32.data_f32 = {0.1f, 0.5f, 1.0f};
  msg.layers = {u8, f32};

// msg -> core
navmap::NavMap core = navmap_ros::from_msg(msg);

// save(core) -> load(core)
std::string path = (std::filesystem::temp_directory_path() /
    ("core_roundtrip_" + std::to_string(::getpid()) + ".navmap")).string();
std::error_code ec;
ASSERT_TRUE(navmap_ros::io::save_to_file(core, path, {}, &ec)) << ec.message();

navmap::NavMap core_loaded;
ASSERT_TRUE(navmap_ros::io::load_from_file(path, core_loaded, &ec)) << ec.message();

// core -> msg
auto msg_from_core = navmap_ros::to_msg(core);
auto msg_from_core_loaded = navmap_ros::to_msg(core_loaded);

// Comparación semántica (tolerante a orden y FP)
ExpectNavMapMsgEqualSemantic(msg_from_core, msg_from_core_loaded);

std::filesystem::remove(path);
  std::filesystem::remove(path);
}
