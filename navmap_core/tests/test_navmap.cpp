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
#include <Eigen/Core>
#include <cstdint>
#include <limits>
#include <vector>
#include <algorithm>
#include <memory>

#include "navmap_core/NavMap.hpp"

using namespace navmap;

namespace
{
constexpr float kEps = 1e-5f;

// Helper: set a triangle (NavCel) vertices.
inline void set_navcel(
  NavMap & nm, NavCelId cid,
  PointId a, PointId b, PointId c)
{
  nm.navcels[cid].v[0] = a;
  nm.navcels[cid].v[1] = b;
  nm.navcels[cid].v[2] = c;
}

// Helper: make a flat square as two triangles on z=0.
inline void make_flat_square(NavMap & nm)
{
  nm.positions.x = {0.0f, 1.0f, 0.0f, 1.0f};
  nm.positions.y = {0.0f, 0.0f, 1.0f, 1.0f};
  nm.positions.z = {0.0f, 0.0f, 0.0f, 0.0f};
  nm.navcels.resize(2);
  set_navcel(nm, 0, 0, 1, 2);
  set_navcel(nm, 1, 2, 1, 3);
  nm.surfaces.resize(1);
  nm.surfaces[0].frame_id = "map";
  nm.surfaces[0].navcels = {0, 1};
  nm.rebuild_geometry_accels();
}
}  // namespace

// ------------------------------ Types & registry ------------------------------

TEST(NavMap_TypesAndLayers, LayerRegistryAddGetList) {
  NavMap nm;

  // Build a trivial mesh with 3 triangles to test sizes
  nm.positions.x = {0, 1, 0, 1, 2, 1};
  nm.positions.y = {0, 0, 1, 0, 0, 1};
  nm.positions.z = {0, 0, 0, 0, 0, 0};
  nm.navcels.resize(3);
  set_navcel(nm, 0, 0, 1, 2);
  set_navcel(nm, 1, 3, 4, 5);
  set_navcel(nm, 2, 0, 2, 3);
  nm.surfaces.resize(1);
  nm.surfaces[0].navcels = {0, 1, 2};
  nm.rebuild_geometry_accels();

  auto occ = nm.layers.add_or_get<uint8_t>("occupancy", nm.navcels.size(), LayerType::U8);
  ASSERT_TRUE(occ);
  EXPECT_EQ(occ->name(), "occupancy");
  EXPECT_EQ(occ->size(), nm.navcels.size());

  auto trav = nm.layers.add_or_get<float>("traversability", nm.navcels.size(), LayerType::F32);
  ASSERT_TRUE(trav);
  EXPECT_EQ(trav->name(), "traversability");
  EXPECT_EQ(trav->size(), nm.navcels.size());

  auto again = std::dynamic_pointer_cast<LayerView<uint8_t>>(nm.layers.get("occupancy"));
  ASSERT_TRUE(again);
  EXPECT_EQ(again.get(), occ.get());

  const auto names = nm.layers.list();
  EXPECT_NE(std::find(names.begin(), names.end(), "occupancy"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "traversability"), names.end());
}

TEST(NavMap_TypesAndLayers, ColorsOptionalPresent) {
  NavMap nm;
  nm.positions.x.resize(3);
  nm.positions.y.resize(3);
  nm.positions.z.resize(3);

  nm.colors.emplace();
  nm.colors->r = {255, 0, 0};
  nm.colors->g = {0, 255, 0};
  nm.colors->b = {0, 0, 255};
  nm.colors->a = {255, 255, 255};

  ASSERT_TRUE(nm.colors.has_value());
  EXPECT_EQ(nm.colors->r.size(), 3u);
  EXPECT_EQ(nm.colors->g.size(), 3u);
  EXPECT_EQ(nm.colors->b.size(), 3u);
  EXPECT_EQ(nm.colors->a.size(), 3u);
}

// ----------------------------- Adjacency & tri-values ------------------------

TEST(NavMap_AdjacencyAndMean, BuildAdjacencyAndTriangleValue) {
  NavMap nm;
  make_flat_square(nm);

  // Neighbors must link across the shared edge.
  auto n0 = nm.get_neighbors(0);
  auto n1 = nm.get_neighbors(1);
  bool link01 = (n0[0] == 1) || (n0[1] == 1) || (n0[2] == 1);
  bool link10 = (n1[0] == 0) || (n1[1] == 0) || (n1[2] == 0);
  EXPECT_TRUE(link01);
  EXPECT_TRUE(link10);

  // Occupancy layer PER-TRIANGLE (uint8_t).
  auto occ = nm.layers.add_or_get<uint8_t>("occupancy", nm.navcels.size(), LayerType::U8);
  (*occ)[0] = 30;
  (*occ)[1] = 200;

  // navcel_value now equals per-triangle value (compat alias).
  const uint8_t val0 = nm.navcel_value<uint8_t>(0, *occ);
  const uint8_t val1 = nm.navcel_value<uint8_t>(1, *occ);
  EXPECT_EQ(val0, 30);
  EXPECT_EQ(val1, 200);

  // Float layer per-triangle.
  auto trav = nm.layers.add_or_get<float>("traversability", nm.navcels.size(), LayerType::F32);
  (*trav)[0] = 0.25f;
  (*trav)[1] = 0.75f;
  EXPECT_NEAR(nm.navcel_value<float>(0, *trav), 0.25f, kEps);
  EXPECT_NEAR(nm.navcel_value<float>(1, *trav), 0.75f, kEps);
}

// ------------------------------- Raycast (flat) ------------------------------

TEST(NavMap_Raycast, FlatFloorDownwardHits) {
  NavMap nm;
  make_flat_square(nm);

  // Downward vertical ray in the middle should hit z=0.
  Eigen::Vector3f o(0.5f, 0.5f, 1.0f);
  Eigen::Vector3f d(0.0f, 0.0f, -1.0f);
  NavCelId cid = 0;
  float t = 0.0f;
  Eigen::Vector3f hit;
  const bool ok = nm.raycast(o, d, cid, t, hit);
  ASSERT_TRUE(ok);
  EXPECT_NEAR(hit.z(), 0.0f, kEps);
  EXPECT_TRUE(cid == 0 || cid == 1);

  // Upward ray from above the floor should miss.
  Eigen::Vector3f d_up(0.0f, 0.0f, 1.0f);
  NavCelId cid_up = 0;
  float t_up = 0.0f;
  Eigen::Vector3f hit_up;
  const bool miss = nm.raycast(o, d_up, cid_up, t_up, hit_up);
  EXPECT_FALSE(miss);
}

// ------------------------------ Locate (flat) -------------------------------

TEST(NavMap_Locate, FlatFloorNoHintUsesRaycast) {
  NavMap nm;
  make_flat_square(nm);

  Eigen::Vector3f p(0.25f, 0.25f, 0.4f);
  size_t sidx = std::numeric_limits<size_t>::max();
  NavCelId cid = std::numeric_limits<uint32_t>::max();
  Eigen::Vector3f bary(0.0f, 0.0f, 0.0f);
  Eigen::Vector3f hit;

  NavMap::LocateOpts opts;
  opts.height_eps = 0.7f;
  const bool ok = nm.locate_navcel(p, sidx, cid, bary, &hit, opts);
  ASSERT_TRUE(ok);
  EXPECT_EQ(sidx, 0u);
  EXPECT_TRUE(cid == 0 || cid == 1);
  EXPECT_NEAR(hit.z(), 0.0f, kEps);
  EXPECT_GE(bary.x(), -1e-4f);
  EXPECT_GE(bary.y(), -1e-4f);
  EXPECT_GE(bary.z(), -1e-4f);
}

TEST(NavMap_Locate, FlatFloorWalkingWithHint) {
  NavMap nm;
  make_flat_square(nm);

  // First locate to get a valid hint.
  Eigen::Vector3f p1(0.1f, 0.1f, 0.3f);
  size_t sidx = 0;
  NavCelId cid = 0;
  Eigen::Vector3f bary(0.0f, 0.0f, 0.0f);
  Eigen::Vector3f hit;
  ASSERT_TRUE(nm.locate_navcel(p1, sidx, cid, bary, &hit));

  // Move slightly to the other triangle. Walking should cross a neighbor.
  Eigen::Vector3f p2(0.75f, 0.75f, 0.3f);
  NavMap::LocateOpts opts;
  opts.hint_cid = cid;
  opts.height_eps = 0.7f;
  NavCelId cid2 = std::numeric_limits<uint32_t>::max();
  Eigen::Vector3f bary2(0.0f, 0.0f, 0.0f);
  Eigen::Vector3f hit2;
  size_t sidx2 = 0;
  ASSERT_TRUE(nm.locate_navcel(p2, sidx2, cid2, bary2, &hit2, opts));
  EXPECT_EQ(sidx2, 0u);
  EXPECT_TRUE(cid2 == 0 || cid2 == 1);
}

// --------------------------- Multi-floor (stacked) ---------------------------

TEST(NavMap_MultiFloor, TwoStackedFloorsLocateToClosest) {
  NavMap nm;

  // Floor 0 at z=0 (same 2-triangle square). Floor 1 at z=3.
  nm.positions.x = {0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f};
  nm.positions.y = {0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f};
  nm.positions.z = {0.0f, 0.0f, 0.0f, 0.0f, 3.0f, 3.0f, 3.0f, 3.0f};

  nm.navcels.resize(4);
  // Floor 0
  set_navcel(nm, 0, 0, 1, 2);
  set_navcel(nm, 1, 2, 1, 3);
  // Floor 1 (offset points by +4)
  set_navcel(nm, 2, 4, 5, 6);
  set_navcel(nm, 3, 6, 5, 7);

  nm.surfaces.resize(2);
  nm.surfaces[0].frame_id = "floor_0";
  nm.surfaces[0].navcels = {0, 1};
  nm.surfaces[1].frame_id = "floor_1";
  nm.surfaces[1].navcels = {2, 3};

  nm.rebuild_geometry_accels();

  // Query just below the upper floor: should select floor_1 by height.
  Eigen::Vector3f p(0.5f, 0.5f, 2.7f);
  size_t sidx = 99;
  NavCelId cid = 999;
  Eigen::Vector3f bary(0.0f, 0.0f, 0.0f);
  Eigen::Vector3f hit;
  NavMap::LocateOpts opts;
  opts.height_eps = 0.6f;
  ASSERT_TRUE(nm.locate_navcel(p, sidx, cid, bary, &hit, opts));
  EXPECT_EQ(sidx, 1u);
  EXPECT_TRUE(cid == 2 || cid == 3);
  EXPECT_NEAR(hit.z(), 3.0f, kEps);
}

// ---------------------------- Non-flat (terrain) ----------------------------

TEST(NavMap_Terrain, SlopedSurfaceLocateAndTriangleValue) {
  NavMap nm;

  // Make a 2x1 rectangle split into two triangles forming a slope on z.
  nm.positions.x = {0.0f, 1.0f, 0.0f, 1.0f};
  nm.positions.y = {0.0f, 0.0f, 1.0f, 1.0f};
  nm.positions.z = {0.0f, 0.2f, 0.4f, 0.6f};  // rising along both axes

  nm.navcels.resize(2);
  set_navcel(nm, 0, 0, 1, 2);
  set_navcel(nm, 1, 2, 1, 3);

  nm.surfaces.resize(1);
  nm.surfaces[0].frame_id = "terrain";
  nm.surfaces[0].navcels = {0, 1};

  nm.rebuild_geometry_accels();

  // Locate above a center point; expect hit near interpolated z.
  Eigen::Vector3f p(0.5f, 0.5f, 2.0f);
  size_t sidx = 0u;
  NavCelId cid = 0;
  Eigen::Vector3f bary(0.0f, 0.0f, 0.0f);
  Eigen::Vector3f hit;
  ASSERT_TRUE(nm.locate_navcel(p, sidx, cid, bary, &hit));
  EXPECT_EQ(sidx, 0u);
  EXPECT_TRUE(cid == 0 || cid == 1);
  // Hit z should be between 0.2 and 0.6 on this slope.
  EXPECT_GT(hit.z(), 0.15f);
  EXPECT_LT(hit.z(), 0.65f);

  // Traversability per-triangle (constant over the triangle).
  auto trav = nm.layers.add_or_get<float>("traversability", nm.navcels.size(), LayerType::F32);
  (*trav)[0] = 0.2f;
  (*trav)[1] = 0.9f;

  const float trav_at_hit = nm.navcel_value<float>(cid, *trav);
  EXPECT_GE(trav_at_hit, 0.0f);
  EXPECT_LE(trav_at_hit, 1.0f);
}

// ------------------------------- Layers: occupancy -----------------------------

TEST(NavMap_Layers, OccupancyPerTriangleAndLookup) {
  NavMap nm;
  make_flat_square(nm);

  // Occupancy (Nav2 style): 0 free, 254 occupied, 255 unknown. Per-triangle.
  auto occ = nm.layers.add_or_get<uint8_t>("occupancy", nm.navcels.size(), LayerType::U8);
  (*occ)[0] = 0;      // free
  (*occ)[1] = 254;    // lethal

  // Locate a point in the second triangle area (roughly).
  Eigen::Vector3f p(0.75f, 0.75f, 0.4f);
  size_t sidx = 0u;
  NavCelId cid = 0u;
  Eigen::Vector3f bary(0.0f, 0.0f, 0.0f);
  Eigen::Vector3f hit;
  ASSERT_TRUE(nm.locate_navcel(p, sidx, cid, bary, &hit));
  ASSERT_TRUE(cid == 0 || cid == 1);
  EXPECT_EQ(nm.navcel_value<uint8_t>(cid, *occ), (cid == 0 ? 0 : 254));
}

// ------------------------------- Edge conditions -----------------------------

TEST(NavMap_Edges, OutOfBoundsAndNoHit) {
  NavMap nm;
  make_flat_square(nm);

  // Far outside XY and above: locate should fail with tight height_eps.
  Eigen::Vector3f p(5.0f, 5.0f, 0.4f);
  size_t sidx = 0u;
  NavCelId cid = 0u;
  Eigen::Vector3f bary(0.0f, 0.0f, 0.0f);
  Eigen::Vector3f hit;
  NavMap::LocateOpts opts;
  opts.height_eps = 0.1f;

  const bool found = nm.locate_navcel(p, sidx, cid, bary, &hit, opts);
  EXPECT_FALSE(found);

  // Raycast from below pointing downward should miss.
  Eigen::Vector3f o(0.5f, 0.5f, -1.0f);
  Eigen::Vector3f d(0.0f, 0.0f, -1.0f);
  NavCelId cid_out = 0u;
  float t = 0.0f;
  Eigen::Vector3f h;
  const bool hit_ok = nm.raycast(o, d, cid_out, t, h);
  EXPECT_FALSE(hit_ok);
}

TEST(NavMap_Layers, HashInvalidationOnElementWrite)
{
  navmap::NavMap a, b;
  make_flat_square(a);
  b = a;

  a.add_layer<uint8_t>("occupancy", "", "", 0);
  a.add_layer<uint8_t>("obstacles", "", "", 0);

  a.layer_set<uint8_t>("obstacles", a.surfaces[0].navcels[0], 254);
  a.layer_set<uint8_t>("obstacles", a.surfaces[0].navcels[1], 254);

  b = a;

  auto la = b.layers.get("occupancy");
  auto lb = b.layers.get("obstacles");
  auto occ = std::dynamic_pointer_cast<navmap::LayerView<uint8_t>>(la);
  auto obs = std::dynamic_pointer_cast<navmap::LayerView<uint8_t>>(lb);
  ASSERT_TRUE(occ && obs);
  size_t diffs = 0;
  for (size_t i = 0; i < occ->size(); ++i) {
    if (occ->data()[i] != obs->data()[i]) {
      ++diffs;
    }
}
  EXPECT_GE(diffs, 2u);
}
