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
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "navmap_ros_interfaces/msg/nav_map_layer.hpp"
#include "navmap_ros_interfaces/msg/nav_map.hpp"

#include "navmap_ros/conversions.hpp"
#include "navmap_core/NavMap.hpp"

using navmap_ros::from_occupancy_grid;
using navmap_ros::to_occupancy_grid;


static void build_square_with_layers(navmap::NavMap & nm)
{
  // Geometry
  std::size_t sidx = nm.create_surface("map");
  uint32_t v0 = nm.add_vertex({0.f, 0.f, 0.f});
  uint32_t v1 = nm.add_vertex({1.f, 0.f, 0.f});
  uint32_t v2 = nm.add_vertex({1.f, 1.f, 0.f});
  uint32_t v3 = nm.add_vertex({0.f, 1.f, 0.f});
  navmap::NavCelId c0 = nm.add_navcel(v0, v1, v2);
  navmap::NavCelId c1 = nm.add_navcel(v0, v2, v3);
  nm.add_navcel_to_surface(sidx, c0);
  nm.add_navcel_to_surface(sidx, c1);
  nm.rebuild_geometry_accels();

  // Layers
  auto occ = nm.add_layer<uint8_t>("occupancy", "Per-NavCel occupancy", "%", uint8_t(0));
  auto cost = nm.add_layer<float>("cost", "Traversal cost", "", 0.0f);
  auto elev = nm.add_layer<double>("elevation", "Elevation", "m", 0.0);

  occ->data()[0] = 42u;   occ->data()[1] = 255u;
  cost->data()[0] = 1.5f; cost->data()[1] = 7.25f;
  elev->data()[0] = 0.0;  elev->data()[1] = 0.0;
}

static nav_msgs::msg::OccupancyGrid make_grid_4m_0p1()
{
  nav_msgs::msg::OccupancyGrid g;
  const int W = 40, H = 40;
  g.header.frame_id = "map";
  g.info.width = W;
  g.info.height = H;
  g.info.resolution = 0.1f;
  g.info.origin.position.x = 0.0;
  g.info.origin.position.y = 0.0;
  g.info.origin.position.z = 0.0;
  g.info.origin.orientation.w = 1.0;
  g.data.assign(W * H, 0);

  auto id = [&](int i, int j) {return j * W + i;};

  // Borders 100
  for (int i = 0; i < W; ++i) {
    g.data[id(i, 0)] = 100; g.data[id(i, H - 1)] = 100;
  }
  for (int j = 0; j < H; ++j) {
    g.data[id(0, j)] = 100; g.data[id(W - 1, j)] = 100;
  }

  // Central 6x6 block 100 (cells [17..22]x[17..22])
  for (int j = 17; j <= 22; ++j) {
    for (int i = 17; i <= 22; ++i) {
      g.data[id(i, j)] = 100;
    }
  }

  // Cross 50
  for (int k = 10; k < 30; ++k) {
    g.data[id(20, k)] = 50; g.data[id(k, 20)] = 50;
  }

  // Unknowns
  g.data[id(5, 5)] = -1;
  g.data[id(34, 17)] = -1;
  return g;
}

static void make_flat_square(navmap::NavMap & nm, float z = 0.0f)
{
  // Build a unit square [0,1]x[0,1] at height z as two triangles
  const auto v0 = nm.add_vertex(Eigen::Vector3f(0.f, 0.f, z));
  const auto v1 = nm.add_vertex(Eigen::Vector3f(1.f, 0.f, z));
  const auto v2 = nm.add_vertex(Eigen::Vector3f(1.f, 1.f, z));
  const auto v3 = nm.add_vertex(Eigen::Vector3f(0.f, 1.f, z));

  const auto c0 = nm.add_navcel(v0, v1, v2);
  const auto c1 = nm.add_navcel(v0, v2, v3);

  const std::size_t s = nm.create_surface("map");
  nm.add_navcel_to_surface(s, c0);
  nm.add_navcel_to_surface(s, c1);

  nm.rebuild_geometry_accels();
}

static uint8_t occ_to_u8(int8_t v)
{
  if (v < 0) {return 255u;}
  if (v >= 100) {return 254u;}
  return static_cast<uint8_t>(std::lround((v / 100.0) * 254.0));
}

static inline navmap::NavCelId tri_index_for_cell(uint32_t i, uint32_t j, uint32_t W)
{
  return static_cast<navmap::NavCelId>((j * W + i) * 2);
}

TEST(NavMap_FullConversions, RoundTrip_All)
{
  navmap::NavMap a; build_square_with_layers(a);
  auto msg = navmap_ros::to_msg(a);
  navmap::NavMap b = navmap_ros::from_msg(msg);

  ASSERT_EQ(a.positions.size(), b.positions.size());
  for (size_t i = 0; i < a.positions.size(); ++i) {
    EXPECT_NEAR(a.positions.x[i], b.positions.x[i], 1e-9);
    EXPECT_NEAR(a.positions.y[i], b.positions.y[i], 1e-9);
    EXPECT_NEAR(a.positions.z[i], b.positions.z[i], 1e-9);
  }
  ASSERT_EQ(a.navcels.size(), b.navcels.size());
  for (size_t i = 0; i < a.navcels.size(); ++i) {
    EXPECT_EQ(a.navcels[i].v[0], b.navcels[i].v[0]);
    EXPECT_EQ(a.navcels[i].v[1], b.navcels[i].v[1]);
    EXPECT_EQ(a.navcels[i].v[2], b.navcels[i].v[2]);
  }
  ASSERT_EQ(a.surfaces.size(), b.surfaces.size());
  for (size_t s = 0; s < a.surfaces.size(); ++s) {
    EXPECT_EQ(a.surfaces[s].frame_id, b.surfaces[s].frame_id);
    ASSERT_EQ(a.surfaces[s].navcels.size(), b.surfaces[s].navcels.size());
    for (size_t k = 0; k < a.surfaces[s].navcels.size(); ++k) {
      EXPECT_EQ(a.surfaces[s].navcels[k], b.surfaces[s].navcels[k]);
    }
  }

  auto names = a.list_layers();
  auto names2 = b.list_layers();
  ASSERT_EQ(names2.size(), names.size());
  for (const auto & lname : names) {
    ASSERT_TRUE(b.has_layer(lname));
    EXPECT_EQ(b.layer_type_name(lname), a.layer_type_name(lname));
    ASSERT_EQ(b.layer_size(lname), a.layer_size(lname));
    for (navmap::NavCelId cid = 0; cid < a.navcels.size(); ++cid) {
      double va = a.layer_get<double>(lname, cid);
      double vb = b.layer_get<double>(lname, cid);
      if (std::isnan(va) || std::isnan(vb)) {
        EXPECT_TRUE(std::isnan(va) && std::isnan(vb));
      } else {
        EXPECT_NEAR(va, vb, 1e-9);
      }
    }
  }
}

TEST(NavMap_FullConversions, EmptyMap_RoundTrip)
{
  navmap::NavMap a;
  auto msg = navmap_ros::to_msg(a);
  navmap::NavMap b = navmap_ros::from_msg(msg);
  EXPECT_EQ(b.positions.size(), 0u);
  EXPECT_EQ(b.navcels.size(), 0u);
  EXPECT_EQ(b.surfaces.size(), 0u);
  EXPECT_TRUE(b.list_layers().empty());
}

TEST(NavMap_LayerConversions, U8_RoundTrip)
{
  navmap::NavMap nm; make_flat_square(nm);
  auto occ = nm.add_layer<uint8_t>("occupancy", "occ", "", uint8_t(0));
  occ->data()[0] = 10u; occ->data()[1] = 250u;

  auto msg = navmap_ros::to_msg(nm, "occupancy");
  EXPECT_EQ(msg.name, "occupancy");
  EXPECT_EQ(msg.type, 0u);  // 0=U8
  ASSERT_EQ(msg.data_u8.size(), 2u);
  EXPECT_EQ(msg.data_u8[0], 10u);
  EXPECT_EQ(msg.data_u8[1], 250u);

  navmap::NavMap nm2; make_flat_square(nm2);
  navmap_ros::from_msg(msg, nm2);
  ASSERT_TRUE(nm2.has_layer("occupancy"));
  EXPECT_NEAR(nm2.layer_get<double>("occupancy", 0), 10.0, 1e-6);
  EXPECT_NEAR(nm2.layer_get<double>("occupancy", 1), 250.0, 1e-6);
}

TEST(NavMap_LayerConversions, F32_RoundTrip)
{
  navmap::NavMap nm; make_flat_square(nm);
  auto cost = nm.add_layer<float>("cost", "cost", "", 0.0f);
  cost->data()[0] = 1.25f; cost->data()[1] = 9.5f;

  auto msg = navmap_ros::to_msg(nm, "cost");
  EXPECT_EQ(msg.type, 1u);  // 1=F32
  ASSERT_EQ(msg.data_f32.size(), 2u);
  EXPECT_NEAR(msg.data_f32[0], 1.25f, 1e-6);
  EXPECT_NEAR(msg.data_f32[1], 9.5f, 1e-6);

  navmap::NavMap nm2; make_flat_square(nm2);
  navmap_ros::from_msg(msg, nm2);
  ASSERT_TRUE(nm2.has_layer("cost"));
  EXPECT_NEAR(nm2.layer_get<double>("cost", 0), 1.25, 1e-6);
  EXPECT_NEAR(nm2.layer_get<double>("cost", 1), 9.5, 1e-6);
}

TEST(NavMap_LayerConversions, F64_RoundTrip)
{
  navmap::NavMap nm; make_flat_square(nm);
  auto elev = nm.add_layer<double>("elevation", "elev", "m", 0.0);
  elev->data()[0] = 12.345;
  elev->data()[1] = -2.5;

  auto msg = navmap_ros::to_msg(nm, "elevation");
  EXPECT_EQ(msg.type, 2u);  // 2=F64
  ASSERT_EQ(msg.data_f64.size(), 2u);
  EXPECT_NEAR(msg.data_f64[0], 12.345, 1e-9);
  EXPECT_NEAR(msg.data_f64[1], -2.5, 1e-9);

  navmap::NavMap nm2; make_flat_square(nm2);
  navmap_ros::from_msg(msg, nm2);
  ASSERT_TRUE(nm2.has_layer("elevation"));
  EXPECT_NEAR(nm2.layer_get<double>("elevation", 0), 12.345, 1e-9);
  EXPECT_NEAR(nm2.layer_get<double>("elevation", 1), -2.5, 1e-9);
}

TEST(NavMap_LayerConversions, ToMsg_Throws_OnMissingLayer)
{
  navmap::NavMap nm; make_flat_square(nm);
  EXPECT_THROW({auto msg = navmap_ros::to_msg(nm, "missing"); (void)msg;}, std::runtime_error);
}

TEST(TestConversions, RoundTrip_ExactEquality_4m_0p1)
{
  const int W = 40, H = 40;
  auto g = make_grid_4m_0p1();

  auto nm = from_occupancy_grid(g);
  auto gout = to_occupancy_grid(nm);

  ASSERT_EQ(gout.info.width, g.info.width);
  ASSERT_EQ(gout.info.height, g.info.height);
  ASSERT_NEAR(gout.info.resolution, g.info.resolution, 1e-7f);

  EXPECT_NEAR(gout.info.origin.position.x, g.info.origin.position.x, 1e-7);
  EXPECT_NEAR(gout.info.origin.position.y, g.info.origin.position.y, 1e-7);
  EXPECT_NEAR(gout.info.origin.position.z, g.info.origin.position.z, 1e-7);
  EXPECT_NEAR(gout.info.origin.orientation.w, g.info.origin.orientation.w, 1e-7);

  ASSERT_EQ(gout.data.size(), g.data.size());
  for (size_t idx = 0; idx < g.data.size(); ++idx) {
    EXPECT_EQ(gout.data[idx], g.data[idx]) << "Mismatch at cell " << idx;
  }

  // Structural checks (shared vertices)
  ASSERT_EQ(nm.positions.size(), static_cast<size_t>((W + 1) * (H + 1)));
  ASSERT_EQ(nm.navcels.size(), static_cast<size_t>(2 * W * H));
  ASSERT_EQ(nm.surfaces.size(), 1u);

  auto base = nm.layers.get("occupancy");
  ASSERT_TRUE(base && base->type() == navmap::LayerType::U8);
  auto occ = std::dynamic_pointer_cast<navmap::LayerView<uint8_t>>(base);
  ASSERT_EQ(occ->size(), nm.navcels.size());

  // Two triangles per cell must carry identical value equal to source cell.
  for (uint32_t j = 0; j < static_cast<uint32_t>(H); ++j) {
    for (uint32_t i = 0; i < static_cast<uint32_t>(W); ++i) {
      const auto cell = j * W + i;
      const uint8_t exp = occ_to_u8(g.data[cell]);
      const navmap::NavCelId t0 = tri_index_for_cell(i, j, W);
      const navmap::NavCelId t1 = t0 + 1;
      ASSERT_LT(t1, nm.navcels.size());
      EXPECT_EQ((*occ)[t0], exp) << "cell(" << i << "," << j << ")";
      EXPECT_EQ((*occ)[t1], exp) << "cell(" << i << "," << j << ")";
    }
  }
}

TEST(TestConversions, TriangleIndicesFollowPattern0)
{
  const int W = 40;
  auto g = make_grid_4m_0p1();
  auto nm = from_occupancy_grid(g);

  // Pick a cell and verify its triangles reference the expected 4 vertices.
  auto v_id = [W](uint32_t i, uint32_t j) -> navmap::PointId {
      return static_cast<navmap::PointId>(j * (W + 1) + i);
    };

  const uint32_t ci = 10, cj = 11;
  const navmap::NavCelId t0 = tri_index_for_cell(ci, cj, W);
  const navmap::NavCelId t1 = t0 + 1;

  ASSERT_LT(t1, nm.navcels.size());

  const auto & a = nm.navcels[t0];
  const auto & b = nm.navcels[t1];

  EXPECT_EQ(a.v[0], v_id(ci + 0, cj + 0));
  EXPECT_EQ(a.v[1], v_id(ci + 1, cj + 0));
  EXPECT_EQ(a.v[2], v_id(ci + 1, cj + 1));

  EXPECT_EQ(b.v[0], v_id(ci + 0, cj + 0));
  EXPECT_EQ(b.v[1], v_id(ci + 1, cj + 1));
  EXPECT_EQ(b.v[2], v_id(ci + 0, cj + 1));
}
