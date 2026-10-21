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

#include <cstdint>
#include <utility>
#include <vector>

#include "navmap_core/NavMap.hpp"

using navmap::NavCelId;
using navmap::NavMap;

namespace
{

// Adds an n x n grid floor at height z as a new surface; returns the surface index.
std::size_t add_floor(NavMap & nm, float z, int n = 10, float res = 1.0f)
{
  const std::size_t s = nm.create_surface("map");
  const auto base = static_cast<uint32_t>(nm.positions.size());
  for (int j = 0; j <= n; ++j) {
    for (int i = 0; i <= n; ++i) {
      nm.add_vertex({i * res, j * res, z});
    }
  }
  auto v = [&](int i, int j) {return base + static_cast<uint32_t>(j * (n + 1) + i);};
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j), v(i + 1, j + 1)));
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j + 1), v(i, j + 1)));
    }
  }
  return s;
}

struct Located
{
  bool ok{false};
  std::size_t surface{0};
  NavCelId cid{0};
  Eigen::Vector3f hit{Eigen::Vector3f::Zero()};
};

Located locate(const NavMap & nm, const Eigen::Vector3f & p, NavMap::LocateOpts opts = {})
{
  Located r;
  Eigen::Vector3f bary;
  r.ok = nm.locate_navcel(p, r.surface, r.cid, bary, &r.hit, opts);
  return r;
}

// Floors at z = 0, 3, 6 listed in the given order.
NavMap build_floors(const std::vector<float> & heights)
{
  NavMap nm;
  for (float z : heights) {
    add_floor(nm, z);
  }
  nm.rebuild_geometry_accels();
  return nm;
}

}  // namespace

TEST(NavMap_LocateMultiSurface, PointJustAboveEachFloorLocatesThatFloor)
{
  const NavMap nm = build_floors({0.0f, 3.0f, 6.0f});
  const float heights[] = {0.0f, 3.0f, 6.0f};
  for (std::size_t k = 0; k < 3; ++k) {
    const auto r = locate(nm, {5.2f, 5.3f, heights[k] + 0.1f});
    ASSERT_TRUE(r.ok) << "floor " << k;
    EXPECT_EQ(r.surface, k) << "floor " << k;
    EXPECT_NEAR(r.hit.z(), heights[k], 1e-5f) << "floor " << k;
    EXPECT_EQ(nm.surface_of(r.cid), k);
  }
}

TEST(NavMap_LocateMultiSurface, PointExactlyOnAFloorLocatesThatFloor)
{
  // Typical robot pose: z exactly on the floor it stands on.
  const NavMap nm = build_floors({0.0f, 3.0f, 6.0f});
  for (std::size_t k = 0; k < 3; ++k) {
    const float z = 3.0f * static_cast<float>(k);
    const auto r = locate(nm, {5.2f, 5.3f, z});
    ASSERT_TRUE(r.ok) << "floor " << k;
    EXPECT_EQ(r.surface, k) << "floor " << k;
    EXPECT_NEAR(r.hit.z(), z, 1e-5f);
  }
  // Single flat map, as built from an OccupancyGrid.
  const NavMap flat = build_floors({0.0f});
  const auto r = locate(flat, {5.2f, 5.3f, 0.0f});
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.surface, 0u);
}

TEST(NavMap_LocateMultiSurface, ResultDoesNotDependOnSurfaceOrder)
{
  // Same building, surfaces listed top-down: the closest floor must still win.
  const NavMap nm = build_floors({6.0f, 3.0f, 0.0f});
  const auto r0 = locate(nm, {5.2f, 5.3f, 0.1f});
  const auto r1 = locate(nm, {5.2f, 5.3f, 3.1f});
  const auto r2 = locate(nm, {5.2f, 5.3f, 6.1f});
  ASSERT_TRUE(r0.ok && r1.ok && r2.ok);
  EXPECT_EQ(r0.surface, 2u);
  EXPECT_EQ(r1.surface, 1u);
  EXPECT_EQ(r2.surface, 0u);
  EXPECT_NEAR(r1.hit.z(), 3.0f, 1e-5f);
}

TEST(NavMap_LocateMultiSurface, BetweenFloorsPicksTheClosestInZ)
{
  const NavMap nm = build_floors({0.0f, 3.0f});
  EXPECT_EQ(locate(nm, {4.5f, 4.5f, 1.0f}).surface, 0u);
  EXPECT_EQ(locate(nm, {4.5f, 4.5f, 2.0f}).surface, 1u);
  EXPECT_EQ(locate(nm, {4.5f, 4.5f, 2.9f}).surface, 1u);
}

TEST(NavMap_LocateMultiSurface, AboveTopAndBelowBottomFloor)
{
  const NavMap nm = build_floors({0.0f, 3.0f});
  const auto above = locate(nm, {4.5f, 4.5f, 10.0f});
  ASSERT_TRUE(above.ok);
  EXPECT_EQ(above.surface, 1u);
  const auto below = locate(nm, {4.5f, 4.5f, -2.0f});
  ASSERT_TRUE(below.ok);
  EXPECT_EQ(below.surface, 0u);
}

TEST(NavMap_LocateMultiSurface, OutsideFootprintFails)
{
  const NavMap nm = build_floors({0.0f, 3.0f});
  EXPECT_FALSE(locate(nm, {50.0f, 50.0f, 0.1f}).ok);
}

TEST(NavMap_LocateMultiSurface, HintOnTheRightFloorIsUsed)
{
  const NavMap nm = build_floors({0.0f, 3.0f});
  const auto seed = locate(nm, {2.2f, 2.3f, 3.1f});
  ASSERT_TRUE(seed.ok);
  ASSERT_EQ(seed.surface, 1u);

  // Sequence of nearby queries reusing the previous cell as hint.
  NavMap::LocateOpts opts;
  NavCelId prev = seed.cid;
  for (int k = 0; k < 20; ++k) {
    opts.hint_cid = prev;
    const auto r = locate(nm, {2.2f + 0.3f * k, 2.3f, 3.05f}, opts);
    ASSERT_TRUE(r.ok) << "step " << k;
    EXPECT_EQ(r.surface, 1u) << "step " << k;
    EXPECT_NEAR(r.hit.z(), 3.0f, 1e-5f);
    prev = r.cid;
  }
}

TEST(NavMap_LocateMultiSurface, HintOnTheOtherFloorIsIgnored)
{
  const NavMap nm = build_floors({0.0f, 3.0f});
  const auto upper = locate(nm, {5.2f, 5.3f, 3.1f});
  ASSERT_TRUE(upper.ok);

  // Hint on the upper floor, query on the lower one, same XY.
  NavMap::LocateOpts opts;
  opts.hint_cid = upper.cid;
  const auto r = locate(nm, {5.2f, 5.3f, 0.1f}, opts);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.surface, 0u);
  EXPECT_NEAR(r.hit.z(), 0.0f, 1e-5f);
}

TEST(NavMap_LocateMultiSurface, HintWalksAcrossNeighborsOnItsSurface)
{
  // A: floor at z=0. B: shelf at z=0.5 over the same area. Query at z=0.3 is
  // closer to B, so only a working neighbor walk from a hint on A returns A.
  NavMap nm;
  const std::size_t a = add_floor(nm, 0.0f);
  const std::size_t b = add_floor(nm, 0.5f);
  nm.rebuild_geometry_accels();
  ASSERT_EQ(locate(nm, {5.5f, 5.5f, 0.3f}).surface, b);

  int checked = 0;
  for (NavCelId hint : nm.surfaces[a].navcels) {
    const Eigen::Vector3f c = nm.navcel_centroid(hint);
    if ((c.head<2>() - Eigen::Vector2f(5.5f, 5.5f)).norm() > 3.0f) {continue;}
    NavMap::LocateOpts opts;
    opts.hint_cid = hint;
    const auto r = locate(nm, {5.5f, 5.5f, 0.3f}, opts);
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.surface, a) << "hint " << hint;
    EXPECT_NEAR(r.hit.z(), 0.0f, 1e-5f);
    ++checked;
  }
  EXPECT_GT(checked, 40);
}

TEST(NavMap_LocateMultiSurface, InvalidHintFallsBackToSearch)
{
  const NavMap nm = build_floors({0.0f, 3.0f});
  NavMap::LocateOpts opts;
  opts.hint_cid = static_cast<NavCelId>(nm.navcels.size() + 100);
  const auto r = locate(nm, {5.2f, 5.3f, 3.1f}, opts);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.surface, 1u);
}

TEST(NavMap_LocateMultiSurface, HintOnCellWithoutSurfaceFallsBackToSearch)
{
  NavMap nm = build_floors({0.0f, 3.0f});
  // A loose triangle at z=3 that belongs to no surface.
  const auto a = nm.add_vertex({20.0f, 20.0f, 3.0f});
  const auto b = nm.add_vertex({21.0f, 20.0f, 3.0f});
  const auto c = nm.add_vertex({20.0f, 21.0f, 3.0f});
  const NavCelId loose = nm.add_navcel(a, b, c);
  nm.rebuild_geometry_accels();
  EXPECT_EQ(nm.surface_of(loose), NavMap::kNoSurface);

  NavMap::LocateOpts opts;
  opts.hint_cid = loose;
  const auto r = locate(nm, {5.2f, 5.3f, 3.1f}, opts);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.surface, 1u);
}

TEST(NavMap_SurfaceOf, IndexMatchesSurfaceLists)
{
  const NavMap nm = build_floors({0.0f, 3.0f, 6.0f});
  for (std::size_t s = 0; s < nm.surfaces.size(); ++s) {
    for (auto cid : nm.surfaces[s].navcels) {
      ASSERT_EQ(nm.surface_of(cid), s);
    }
  }
  EXPECT_EQ(nm.surface_of(static_cast<NavCelId>(nm.navcels.size())), NavMap::kNoSurface);
}

TEST(NavMap_SurfaceOf, EmptyMap)
{
  NavMap nm;
  nm.rebuild_geometry_accels();
  EXPECT_EQ(nm.surface_of(0), NavMap::kNoSurface);
}

TEST(NavMap_SurfaceOf, StaleIndexFallsBackToLinearSearch)
{
  NavMap nm = build_floors({0.0f});
  // Add a new surface and cell without rebuilding.
  const std::size_t s = nm.create_surface("map");
  const auto a = nm.add_vertex({20.0f, 20.0f, 0.0f});
  const auto b = nm.add_vertex({21.0f, 20.0f, 0.0f});
  const auto c = nm.add_vertex({20.0f, 21.0f, 0.0f});
  const NavCelId cid = nm.add_navcel(a, b, c);
  nm.add_navcel_to_surface(s, cid);

  EXPECT_EQ(nm.surface_of(cid), s);
  EXPECT_EQ(nm.surface_of(0), 0u);

  nm.rebuild_geometry_accels();
  EXPECT_EQ(nm.surface_of(cid), s);
}

TEST(NavMap_SurfaceOf, SurvivesCopyAssignAndMove)
{
  const NavMap src = build_floors({0.0f, 3.0f});
  const NavCelId upper_cid = src.surfaces[1].navcels.front();

  const NavMap copied(src);
  EXPECT_EQ(copied.surface_of(upper_cid), 1u);

  NavMap assigned;
  assigned = src;
  EXPECT_EQ(assigned.surface_of(upper_cid), 1u);

  // Same-geometry assignment keeps the destination index.
  NavMap same = build_floors({0.0f, 3.0f});
  same = src;
  EXPECT_EQ(same.surface_of(upper_cid), 1u);

  NavMap moved_from = src;
  NavMap moved;
  moved = std::move(moved_from);
  EXPECT_EQ(moved.surface_of(upper_cid), 1u);
  const auto r = locate(moved, {5.2f, 5.3f, 3.1f});
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.surface, 1u);
}

TEST(NavMap_LocateMultiSurface, SetAreaOnUpperFloorLeavesLowerFloorUntouched)
{
  NavMap nm = build_floors({0.0f, 3.0f});
  nm.add_layer<uint8_t>("occupancy", "", "", 0);
  ASSERT_TRUE(
    nm.set_area<uint8_t>(
      {5.0f, 5.0f, 3.1f}, 254, "occupancy",
      navmap::AreaShape::CIRCULAR, 1.5f));

  int marked_lower = 0, marked_upper = 0;
  for (auto cid : nm.surfaces[0].navcels) {
    marked_lower += nm.layer_get<uint8_t>("occupancy", cid) == 254;
  }
  for (auto cid : nm.surfaces[1].navcels) {
    marked_upper += nm.layer_get<uint8_t>("occupancy", cid) == 254;
  }
  EXPECT_EQ(marked_lower, 0);
  EXPECT_GT(marked_upper, 0);
}

TEST(NavMap_LocateMultiSurface, SampleLayerAtReadsTheRightFloor)
{
  NavMap nm = build_floors({0.0f, 3.0f});
  auto layer = nm.add_layer<float>("elevation", "", "m", 0.0f);
  for (std::size_t s = 0; s < nm.surfaces.size(); ++s) {
    for (auto cid : nm.surfaces[s].navcels) {
      (*layer)[cid] = s == 0 ? 0.0f : 3.0f;
    }
  }
  EXPECT_FLOAT_EQ(nm.sample_layer_at("elevation", {5.2f, 5.3f, 0.1f}), 0.0f);
  EXPECT_FLOAT_EQ(nm.sample_layer_at("elevation", {5.2f, 5.3f, 3.1f}), 3.0f);
}
