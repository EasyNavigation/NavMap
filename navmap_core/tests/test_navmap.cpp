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

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cstdint>
#include <limits>
#include <vector>
#include <algorithm>

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

// Helper: barycentric interpolation for a scalar layer on a located point.
template<typename T>
float interp_layer_bary(
  const NavMap & nm,
  const LayerView<T> & layer,
  NavCelId cid,
  const Eigen::Vector3f & bary)
{
  const auto & cel = nm.navcels[cid];
  const float v0 = static_cast<float>(layer[cel.v[0]]);
  const float v1 = static_cast<float>(layer[cel.v[1]]);
  const float v2 = static_cast<float>(layer[cel.v[2]]);
  return bary.x() * v0 + bary.y() * v1 + bary.z() * v2;
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
  nm.positions.x.resize(5);
  nm.positions.y.resize(5);
  nm.positions.z.resize(5);

  auto occ = nm.layers.add_or_get<uint8_t>("occupancy", 5, LayerType::U8);
  ASSERT_TRUE(occ);
  EXPECT_EQ(occ->name(), "occupancy");
  EXPECT_EQ(occ->size(), 5u);

  auto trav = nm.layers.add_or_get<float>("traversability", 5, LayerType::F32);
  ASSERT_TRUE(trav);
  EXPECT_EQ(trav->name(), "traversability");
  EXPECT_EQ(trav->size(), 5u);

  auto again = std::dynamic_pointer_cast<LayerView<uint8_t>>(
      nm.layers.get("occupancy"));
  ASSERT_TRUE(again);
  EXPECT_EQ(again.get(), occ.get());

  const auto names = nm.layers.list();
  EXPECT_NE(std::find(names.begin(), names.end(), "occupancy"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "traversability"),
            names.end());
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

// ----------------------------- Adjacency & mean ------------------------------

TEST(NavMap_AdjacencyAndMean, BuildAdjacencyAndNavCelMean) {
  NavMap nm;
  make_flat_square(nm);

  // Neighbors must link across the shared edge.
  auto n0 = nm.get_neighbors(0);
  auto n1 = nm.get_neighbors(1);
  bool link01 = (n0[0] == 1) || (n0[1] == 1) || (n0[2] == 1);
  bool link10 = (n1[0] == 0) || (n1[1] == 0) || (n1[2] == 0);
  EXPECT_TRUE(link01);
  EXPECT_TRUE(link10);

  // Add an occupancy layer (uint8_t). Use small values to avoid overflow.
  auto occ = nm.layers.add_or_get<uint8_t>("occupancy", 4, LayerType::U8);
  (*occ)[0] = 30;
  (*occ)[1] = 60;
  (*occ)[2] = 90;
  (*occ)[3] = 120;

  // navcel_mean<uint8_t> returns uint8_t. With the chosen values it is safe.
  const uint8_t mean0 = nm.navcel_mean<uint8_t>(0, *occ);
  const uint8_t mean1 = nm.navcel_mean<uint8_t>(1, *occ);
  EXPECT_EQ(mean0, static_cast<uint8_t>((30 + 60 + 90) / 3));
  EXPECT_EQ(mean1, static_cast<uint8_t>((90 + 60 + 120) / 3));

  // Also test a float layer mean for precision.
  auto trav = nm.layers.add_or_get<float>("traversability", 4, LayerType::F32);
  (*trav)[0] = 0.1f;
  (*trav)[1] = 0.3f;
  (*trav)[2] = 0.7f;
  (*trav)[3] = 1.0f;
  const float fmean0 = nm.navcel_mean<float>(0, *trav);
  const float fmean1 = nm.navcel_mean<float>(1, *trav);
  EXPECT_NEAR(fmean0, (0.1f + 0.3f + 0.7f) / 3.0f, kEps);
  EXPECT_NEAR(fmean1, (0.7f + 0.3f + 1.0f) / 3.0f, kEps);
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

  // Floor 0 at z=0 (same 2-triangle square).
  nm.positions.x = {0.0f, 1.0f, 0.0f, 1.0f,
    0.0f, 1.0f, 0.0f, 1.0f};
  nm.positions.y = {0.0f, 0.0f, 1.0f, 1.0f,
    0.0f, 0.0f, 1.0f, 1.0f};
  nm.positions.z = {0.0f, 0.0f, 0.0f, 0.0f,
    3.0f, 3.0f, 3.0f, 3.0f};                  // Floor 1 at z=3

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

TEST(NavMap_Terrain, SlopedSurfaceLocateAndInterpolation) {
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

  // Add traversability and check barycentric interpolation at the hit.
  auto trav = nm.layers.add_or_get<float>("traversability", 4, LayerType::F32);
  (*trav)[0] = 0.0f;
  (*trav)[1] = 0.5f;
  (*trav)[2] = 0.8f;
  (*trav)[3] = 1.0f;

  const float trav_pt = interp_layer_bary<float>(nm, *trav, cid, bary);
  EXPECT_GE(trav_pt, 0.0f);
  EXPECT_LE(trav_pt, 1.0f);
}

// ----------------------------- Layers: occupancy -----------------------------

TEST(NavMap_Layers, OccupancySingleAndMultiLayer) {
  NavMap nm;
  make_flat_square(nm);

  // Occupancy (Nav2 style): 0 free, 254 occupied, 255 unknown.
  auto occ = nm.layers.add_or_get<uint8_t>("occupancy", 4, LayerType::U8);
  (*occ)[0] = 0;      // free
  (*occ)[1] = 100;    // mid
  (*occ)[2] = 254;    // lethal
  (*occ)[3] = 255;    // unknown

  // Means in uint8 (safe sums here are <256 for triangle 0).
  const uint8_t occ_mean0 = nm.navcel_mean<uint8_t>(0, *occ);
  EXPECT_EQ(occ_mean0, static_cast<uint8_t>((0 + 100 + 254) / 3));

  // Multi-layer: add color and traversability.
  nm.colors.emplace();
  nm.colors->r = {10, 20, 30, 40};
  nm.colors->g = {50, 60, 70, 80};
  nm.colors->b = {90, 100, 110, 120};
  nm.colors->a = {255, 255, 255, 255};

  auto trav = nm.layers.add_or_get<float>("traversability", 4, LayerType::F32);
  (*trav)[0] = 0.3f;
  (*trav)[1] = 0.6f;
  (*trav)[2] = 0.9f;
  (*trav)[3] = 0.2f;

  // Locate a point and interpolate traversability there.
  Eigen::Vector3f p(0.75f, 0.25f, 0.4f);
  size_t sidx = 0u;
  NavCelId cid = 0u;
  Eigen::Vector3f bary(0.0f, 0.0f, 0.0f);
  Eigen::Vector3f hit;
  ASSERT_TRUE(nm.locate_navcel(p, sidx, cid, bary, &hit));

  const float trav_at_hit = interp_layer_bary<float>(nm, *trav, cid, bary);
  EXPECT_GE(trav_at_hit, 0.0f);
  EXPECT_LE(trav_at_hit, 1.0f);
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
