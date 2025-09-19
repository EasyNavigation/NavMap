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

  double v0 = nm.layer_get_as_double("cost", nm.surfaces[0].navcels[0]);
  double v1 = nm.layer_get_as_double("cost", nm.surfaces[0].navcels[1]);
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

  // Access to nonexistent layer → NaN
  double val = nm.layer_get_as_double("foo", 0);
  EXPECT_TRUE(std::isnan(val));

  // Type mismatch: set as float, read as double is ok via as_double
  nm.add_layer<float>("speed");
  nm.layer_set<float>("speed", 0, 3.14f);
  double d = nm.layer_get_as_double("speed", 0);
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
  std::size_t idx1 = nm.add_surface(s);      // by copy
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
