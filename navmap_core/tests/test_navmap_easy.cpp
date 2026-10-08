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
#include <algorithm>
#include <string>
#include <vector>
#include <limits>

#include "navmap_core/NavMap.hpp"

using navmap::NavMap;
using navmap::NavCelId;
using navmap::Surface;
using navmap::LayerView;

static void make_flat_square(NavMap & nm, float z = 0.0f)
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

static void make_two_floors(NavMap & nm, float z0, float z1)
{
  make_flat_square(nm, z0);

  // Second floor must be a separate surface
  const auto v0 = nm.add_vertex(Eigen::Vector3f(0.f, 0.f, z1));
  const auto v1 = nm.add_vertex(Eigen::Vector3f(1.f, 0.f, z1));
  const auto v2 = nm.add_vertex(Eigen::Vector3f(1.f, 1.f, z1));
  const auto v3 = nm.add_vertex(Eigen::Vector3f(0.f, 1.f, z1));
  const auto c0 = nm.add_navcel(v0, v1, v2);
  const auto c1 = nm.add_navcel(v0, v2, v3);
  const std::size_t s = nm.create_surface("map");
  nm.add_navcel_to_surface(s, c0);
  nm.add_navcel_to_surface(s, c1);

  nm.rebuild_geometry_accels();
}

// ------------------------- Tests -------------------------

TEST(NavMap_EasyAPI, ConstructionBasics)
{
  NavMap nm;
  Surface s = nm.create_surface_obj("map");
  ASSERT_EQ(s.navcels.size(), 0u);
  const std::size_t sidx = nm.add_surface(s);
  EXPECT_EQ(sidx, 0u);
  EXPECT_EQ(nm.surfaces.size(), 1u);
  EXPECT_EQ(nm.surfaces[0].frame_id, "map");

  auto v0 = nm.add_vertex({0, 0, 0});
  auto v1 = nm.add_vertex({1, 0, 0});
  auto v2 = nm.add_vertex({0, 1, 0});
  auto c0 = nm.add_navcel(v0, v1, v2);
  nm.add_navcel_to_surface(sidx, c0);
  nm.rebuild_geometry_accels();

  EXPECT_EQ(nm.positions.size(), 3u);
  EXPECT_EQ(nm.navcels.size(), 1u);
  EXPECT_EQ(nm.surfaces[0].navcels.size(), 1u);
}

TEST(NavMap_EasyAPI, LayersBasics)
{
  NavMap nm;
  make_flat_square(nm);

  auto cost = nm.add_layer<float>("cost", "Traversal cost", "");
  ASSERT_TRUE(nm.has_layer("cost"));
  EXPECT_EQ(nm.layer_type_name("cost"), "float");
  EXPECT_EQ(nm.layer_size("cost"), nm.navcels.size());

  nm.layer_set<float>("cost", nm.surfaces[0].navcels[0], 1.5f);
  nm.layer_set<float>("cost", nm.surfaces[0].navcels[1], 2.0f);

  double v0 = nm.layer_get<double>("cost", nm.surfaces[0].navcels[0],
                                    std::numeric_limits<double>::quiet_NaN());
  double v1 = nm.layer_get<double>("cost", nm.surfaces[0].navcels[1],
                                    std::numeric_limits<double>::quiet_NaN());
  EXPECT_NEAR(v0, 1.5, 1e-6);
  EXPECT_NEAR(v1, 2.0, 1e-6);

  auto names = nm.list_layers();
  ASSERT_FALSE(names.empty());
  EXPECT_NE(std::find(names.begin(), names.end(), "cost"), names.end());
}

TEST(NavMap_EasyAPI, LayersNegativeCases)
{
  NavMap nm;
  make_flat_square(nm);

  // Access to nonexistent layer → NaN (using explicit default)
  double val = nm.layer_get<double>("foo", 0, std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(std::isnan(val));

  // Type mismatch: set as float, read as double (conversion path)
  nm.add_layer<float>("speed");
  nm.layer_set<float>("speed", 0, 3.14f);
  double d = nm.layer_get<double>("speed", 0, std::numeric_limits<double>::quiet_NaN());
  EXPECT_NEAR(d, 3.14, 1e-6);

  // layer_type_name on nonexistent layer → "unknown"
  EXPECT_EQ(nm.layer_type_name("idontexist"), "unknown");

  // Empty layer → size 0
  EXPECT_EQ(nm.layer_size("idontexist"), 0u);
}

TEST(NavMap_EasyAPI, CentroidAndNeighbors)
{
  NavMap nm;
  make_flat_square(nm);

  const auto c0 = nm.surfaces[0].navcels[0];
  Eigen::Vector3f cc0 = nm.navcel_centroid(c0);

  EXPECT_NEAR(cc0.z(), 0.0f, 1e-6);
  auto n0 = nm.navcel_neighbors(c0);
  EXPECT_FALSE(n0.empty());
}

TEST(NavMap_EasyAPI, LocateWithinSquare)
{
  NavMap nm;
  make_flat_square(nm);

  Eigen::Vector3f p(0.25f, 0.25f, 0.5f);
  size_t sidx{}; NavCelId cid{}; Eigen::Vector3f bary; Eigen::Vector3f hit;
  bool ok = nm.locate_navcel(p, sidx, cid, bary, &hit);
  ASSERT_TRUE(ok);
  EXPECT_EQ(sidx, 0u);
  EXPECT_NEAR(hit.x(), p.x(), 1e-5);
  EXPECT_NEAR(hit.y(), p.y(), 1e-5);
  EXPECT_NEAR(hit.z(), 0.0f, 1e-5);
}

TEST(NavMap_EasyAPI, LocateOutOfBoundsRespectsHeightEps)
{
  NavMap nm;
  make_flat_square(nm);

  Eigen::Vector3f p(5.0f, 5.0f, 0.4f);
  size_t sidx{}; NavCelId cid{}; Eigen::Vector3f bary; Eigen::Vector3f hit;
  NavMap::LocateOpts opts; opts.height_eps = 0.1f;
  bool ok = nm.locate_navcel(p, sidx, cid, bary, &hit, opts);
  EXPECT_FALSE(ok);
}

TEST(NavMap_EasyAPI, LocateMultiFloorChoosesClosestByDz)
{
  NavMap nm;
  make_two_floors(nm, 0.0f, 3.0f);

  Eigen::Vector3f p(0.5f, 0.5f, 2.9f);
  size_t sidx{}; NavCelId cid{}; Eigen::Vector3f bary; Eigen::Vector3f hit;
  bool ok = nm.locate_navcel(p, sidx, cid, bary, &hit);
  ASSERT_TRUE(ok);
  EXPECT_EQ(sidx, 1u);
  EXPECT_NEAR(hit.z(), 3.0f, 1e-4);
}

TEST(NavMap_EasyAPI, SampleLayerAt)
{
  NavMap nm;
  make_flat_square(nm);

  auto cost = nm.add_layer<float>("cost");
  nm.layer_set<float>("cost", nm.surfaces[0].navcels[0], 1.0f);
  nm.layer_set<float>("cost", nm.surfaces[0].navcels[1], 5.0f);

  double vA = nm.sample_layer_at("cost", Eigen::Vector3f(0.2f, 0.2f, 1.0f), -1.0);
  double vB = nm.sample_layer_at("cost", Eigen::Vector3f(0.2f, 0.8f, 1.0f), -1.0);

  EXPECT_NEAR(vA, 1.0, 1e-6);
  EXPECT_NEAR(vB, 5.0, 1e-6);

  // Missing layer → default
  double vC = nm.sample_layer_at("missing", Eigen::Vector3f(0.2f, 0.2f, 1.0f), -42.0);
  EXPECT_EQ(vC, -42.0);
}

TEST(NavMap_EasyAPI, RemoveSurfaceDoesNotBreakContiguousData)
{
  NavMap nm;
  make_two_floors(nm, 0.0f, 3.0f);
  ASSERT_EQ(nm.surfaces.size(), 2u);

  bool removed = nm.remove_surface(0);
  EXPECT_TRUE(removed);
  EXPECT_EQ(nm.surfaces.size(), 1u);
}

TEST(NavMap_EasyAPI, AddSurfaceMoveOverload)
{
  NavMap nm;
  Surface s = nm.create_surface_obj("map");
  s.navcels.clear();
  std::size_t idx1 = nm.add_surface(s);           // by copy
  std::size_t idx2 = nm.add_surface(std::move(s)); // by move
  EXPECT_EQ(idx1, 0u);
  EXPECT_EQ(idx2, 1u);
  EXPECT_EQ(nm.surfaces.size(), 2u);
}

TEST(NavMap_EasyAPI, RaycastVerticalHitAndMiss)
{
  NavMap nm;
  make_flat_square(nm);

  Eigen::Vector3f o(0.5f, 0.5f, 10.0f);
  Eigen::Vector3f d(0.0f, 0.0f, -1.0f);
  NavCelId cid{}; float t{}; Eigen::Vector3f h;
  bool hit = nm.raycast(o, d, cid, t, h);
  EXPECT_TRUE(hit);
  EXPECT_NEAR(h.z(), 0.0f, 1e-4);

  Eigen::Vector3f o2(0.5f, 0.5f, -1.0f);
  Eigen::Vector3f d2(0.0f, 0.0f, -1.0f);
  NavCelId cid2{}; float t2{}; Eigen::Vector3f h2;
  bool hit2 = nm.raycast(o2, d2, cid2, t2, h2);
  EXPECT_FALSE(hit2);
}

// ------------------------- Area-setting tests -------------------------

TEST(NavMap_SetArea, CircularMarksBothTrianglesOnUnitSquare)
{
  NavMap nm;
  make_flat_square(nm);

  // Create obstacles layer as U8, default 0
  nm.add_layer<uint8_t>("obstacles", "occupancy obstacles", "%", 0);

  // Center at (0.5,0.5), radius 0.3 → both triangle centroids are inside
  const bool ok = nm.set_area<uint8_t>(
    Eigen::Vector3f(0.5f, 0.5f, 10.0f),
    static_cast<uint8_t>(254),
    "obstacles",
    navmap::AreaShape::CIRCULAR,
    0.3f);

  ASSERT_TRUE(ok);

  const auto c0 = nm.surfaces[0].navcels[0];
  const auto c1 = nm.surfaces[0].navcels[1];
  uint8_t v0 = nm.layer_get<uint8_t>("obstacles", c0, 0);
  uint8_t v1 = nm.layer_get<uint8_t>("obstacles", c1, 0);

  EXPECT_EQ(v0, static_cast<uint8_t>(254));
  EXPECT_EQ(v1, static_cast<uint8_t>(254));
}

TEST(NavMap_SetArea, RectangularCanAffectSingleTriangle)
{
  NavMap nm;
  make_flat_square(nm);

  nm.add_layer<uint8_t>("obstacles", "occupancy obstacles", "%", 0);

  // Pick a center closer to the first triangle's centroid (2/3, 1/3)
  // Use side length 0.35 (half=0.175) so only that centroid falls inside.
  const bool ok = nm.set_area<uint8_t>(
    Eigen::Vector3f(0.80f, 0.20f, -5.0f),
    static_cast<uint8_t>(200),
    "obstacles",
    navmap::AreaShape::RECTANGULAR,
    0.35f);

  ASSERT_TRUE(ok);

  const auto c0 = nm.surfaces[0].navcels[0];
  const auto c1 = nm.surfaces[0].navcels[1];
  uint8_t v0 = nm.layer_get<uint8_t>("obstacles", c0, 0);
  uint8_t v1 = nm.layer_get<uint8_t>("obstacles", c1, 0);

  EXPECT_EQ(v0, static_cast<uint8_t>(200));
  EXPECT_EQ(v1, static_cast<uint8_t>(0));
}

TEST(NavMap_SetArea, ReturnsFalseWhenSeedCannotBeLocated)
{
  NavMap nm;
  make_flat_square(nm);

  nm.add_layer<uint8_t>("obstacles", "occupancy obstacles", "%", 0);

  // Far away from the mesh; default locator should fail to find a seed.
  const bool ok = nm.set_area<uint8_t>(
    Eigen::Vector3f(5.0f, 5.0f, 0.0f),
    static_cast<uint8_t>(123),
    "obstacles",
    navmap::AreaShape::CIRCULAR,
    1.0f);

  EXPECT_FALSE(ok);

  // Ensure layer remains at default
  const auto c0 = nm.surfaces[0].navcels[0];
  const auto c1 = nm.surfaces[0].navcels[1];
  EXPECT_EQ(nm.layer_get<uint8_t>("obstacles", c0, 0), 0);
  EXPECT_EQ(nm.layer_get<uint8_t>("obstacles", c1, 0), 0);
}

TEST(NavMap_SetArea, TypeMismatchReturnsFalseAndDoesNotModifyData)
{
  NavMap nm;
  make_flat_square(nm);

  // Create "obstacles" as float by mistake
  nm.add_layer<float>("obstacles", "wrong type", "", 0.0f);

  // Try to set it as uint8_t → should fail (type check)
  const bool ok = nm.set_area<uint8_t>(
    Eigen::Vector3f(0.5f, 0.5f, 1.0f),
    static_cast<uint8_t>(254),
    "obstacles",
    navmap::AreaShape::CIRCULAR,
    0.4f);

  EXPECT_FALSE(ok);

  // Verify it was not changed (still float zeros)
  const auto c0 = nm.surfaces[0].navcels[0];
  const auto c1 = nm.surfaces[0].navcels[1];
  float f0 = nm.layer_get<float>("obstacles", c0, -1.0f);
  float f1 = nm.layer_get<float>("obstacles", c1, -1.0f);
  EXPECT_NEAR(f0, 0.0f, 1e-6);
  EXPECT_NEAR(f1, 0.0f, 1e-6);
}

// ------------------------- Systematic area-setting verification -------------------------

static void make_grid(NavMap & nm, int nx, int ny, float z = 0.0f)
{
  // Build a [0,1]x[0,1] plane subdivided in nx*ny quads, 2 triangles per quad
  nm.create_surface("map"); // index 0
  auto addv = [&](float x, float y) {
      return nm.add_vertex(Eigen::Vector3f(x, y, z));
    };

  std::vector<std::vector<uint32_t>> vid((ny + 1), std::vector<uint32_t>(nx + 1));
  for (int j = 0; j <= ny; ++j) {
    for (int i = 0; i <= nx; ++i) {
      float x = static_cast<float>(i) / nx;
      float y = static_cast<float>(j) / ny;
      vid[j][i] = addv(x, y);
    }
  }

  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      uint32_t v00 = vid[j][i];
      uint32_t v10 = vid[j][i + 1];
      uint32_t v01 = vid[j + 1][i];
      uint32_t v11 = vid[j + 1][i + 1];
      // Diagonal v00->v11
      auto c0 = nm.add_navcel(v00, v10, v11);
      auto c1 = nm.add_navcel(v00, v11, v01);
      nm.add_navcel_to_surface(0, c0);
      nm.add_navcel_to_surface(0, c1);
    }
  }

  nm.rebuild_geometry_accels();
}

static bool centroid_inside_circle(const Eigen::Vector3f & c, float cx, float cy, float r)
{
  const float dx = c.x() - cx;
  const float dy = c.y() - cy;
  return (dx * dx + dy * dy) <= (r * r);
}

static bool centroid_inside_square(const Eigen::Vector3f & c, float cx, float cy, float side)
{
  const float half = 0.5f * side;
  const float dx = std::abs(c.x() - cx);
  const float dy = std::abs(c.y() - cy);
  return (dx <= half) && (dy <= half);
}

static void verify_area_by_centroids_u8(
  const NavMap & nm,
  const char *layer,
  uint8_t expected_value,
  std::function<bool(const Eigen::Vector3f &)> inside_pred)
{
  size_t mismatches = 0;
  for (auto cid : nm.surfaces[0].navcels) {
    Eigen::Vector3f cc = nm.navcel_centroid(cid);
    const bool expected_inside = inside_pred(cc);
    const uint8_t v = nm.layer_get<uint8_t>(layer, cid, 0u);
    if (expected_inside) {
      if (v != expected_value) {++mismatches;}
    } else {
      if (v != 0u) {++mismatches;}
    }
  }
  EXPECT_EQ(mismatches, 0u);
}

TEST(NavMap_SetArea_Systematic, CircularOnGridByCentroids)
{
  NavMap nm;
  make_grid(nm, /*nx=*/20, /*ny=*/20);

  // Obstacles layer U8 to 0
  nm.add_layer<uint8_t>("obstacles", "occupancy obstacles", "%", 0);

  // Circle centered at (0.5, 0.5), radius 0.22 → afecta a una vecindad razonable
  const float cx = 0.5f, cy = 0.5f, r = 0.22f;
  const bool ok = nm.set_area<uint8_t>(
    Eigen::Vector3f(cx, cy, 1.0f), static_cast<uint8_t>(254),
    "obstacles", navmap::AreaShape::CIRCULAR, r);
  ASSERT_TRUE(ok);

  verify_area_by_centroids_u8(
    nm, "obstacles", static_cast<uint8_t>(254),
    [&](const Eigen::Vector3f & cc){return centroid_inside_circle(cc, cx, cy, r);});
}

TEST(NavMap_SetArea_Systematic, RectangularOnGridByCentroids)
{
  NavMap nm;
  make_grid(nm, /*nx=*/24, /*ny=*/24);
  nm.add_layer<uint8_t>("obstacles", "occupancy obstacles", "%", 0);

  // Square centered off-center to avoid simetrías
  const float cx = 0.35f, cy = 0.7f, side = 0.18f;
  const bool ok = nm.set_area<uint8_t>(
    Eigen::Vector3f(cx, cy, -2.0f), static_cast<uint8_t>(200),
    "obstacles", navmap::AreaShape::RECTANGULAR, side);
  ASSERT_TRUE(ok);

  verify_area_by_centroids_u8(
    nm, "obstacles", static_cast<uint8_t>(200),
    [&](const Eigen::Vector3f & cc){return centroid_inside_square(cc, cx, cy, side);});
}

TEST(NavMap_SetArea_Systematic, CircularNearBoundary)
{
  NavMap nm;
  make_grid(nm, /*nx=*/30, /*ny=*/30);
  nm.add_layer<uint8_t>("obstacles", "occupancy obstacles", "%", 0);

  // Centro cerca del borde (evita que el BFS salga del mapa)
  const float cx = 0.05f, cy = 0.10f, r = 0.12f;
  const bool ok = nm.set_area<uint8_t>(
    Eigen::Vector3f(cx, cy, 0.0f), static_cast<uint8_t>(180),
    "obstacles", navmap::AreaShape::CIRCULAR, r);
  ASSERT_TRUE(ok);

  verify_area_by_centroids_u8(
    nm, "obstacles", static_cast<uint8_t>(180),
    [&](const Eigen::Vector3f & cc){return centroid_inside_circle(cc, cx, cy, r);});
}
