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

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

#include "navmap_core/NavMap.hpp"

using navmap::NavCelId;
using navmap::NavMap;

namespace
{

constexpr NavCelId kNone = std::numeric_limits<NavCelId>::max();

// n triangles sharing the edge (0,1), each with its own apex.
NavMap fan_on_edge(int n)
{
  NavMap nm;
  const std::size_t s = nm.create_surface("map");
  nm.add_vertex({0.0f, 0.0f, 0.0f});
  nm.add_vertex({1.0f, 0.0f, 0.0f});
  for (int k = 0; k < n; ++k) {
    const float a = 0.5f + static_cast<float>(k);
    const auto apex = nm.add_vertex({0.5f, a, 0.1f * static_cast<float>(k)});
    nm.add_navcel_to_surface(s, nm.add_navcel(0, 1, apex));
  }
  nm.rebuild_geometry_accels();
  return nm;
}

void expect_symmetric(const NavMap & nm)
{
  for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
    for (NavCelId n : nm.navcel_neighbors(cid)) {
      const auto back = nm.navcel_neighbors(n);
      EXPECT_NE(std::find(back.begin(), back.end(), cid), back.end());
    }
  }
}

}  // namespace

TEST(NavMap_Adjacency, TwoCellsOnAnEdgeAreNeighbors)
{
  const NavMap nm = fan_on_edge(2);
  EXPECT_EQ(nm.nonmanifold_edges(), 0u);
  EXPECT_EQ(nm.navcel_neighbors(0), std::vector<NavCelId>{1});
  EXPECT_EQ(nm.navcel_neighbors(1), std::vector<NavCelId>{0});
}

TEST(NavMap_Adjacency, NonManifoldEdgeStaysOpenAndIsCounted)
{
  for (int n : {3, 4, 6}) {
    const NavMap nm = fan_on_edge(n);
    EXPECT_EQ(nm.nonmanifold_edges(), 1u) << n;
    for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
      EXPECT_TRUE(nm.navcel_neighbors(cid).empty()) << n << " cell " << cid;
    }
  }
}

TEST(NavMap_Adjacency, OnlyTheNonManifoldEdgeIsAffected)
{
  // Unit square split in two, plus a third triangle on the diagonal.
  NavMap nm;
  const std::size_t s = nm.create_surface("map");
  const auto a = nm.add_vertex({0, 0, 0});
  const auto b = nm.add_vertex({1, 0, 0});
  const auto c = nm.add_vertex({1, 1, 0});
  const auto d = nm.add_vertex({0, 1, 0});
  const auto e = nm.add_vertex({0.5f, 0.5f, 1.0f});
  const auto f = nm.add_vertex({2, 0, 0});
  const NavCelId t0 = nm.add_navcel(a, b, c);
  const NavCelId t1 = nm.add_navcel(a, c, d);
  const NavCelId t2 = nm.add_navcel(a, c, e);  // third cell on edge a-c
  const NavCelId t3 = nm.add_navcel(b, f, c);  // regular neighbor of t0 on b-c
  for (auto t : {t0, t1, t2, t3}) {
    nm.add_navcel_to_surface(s, t);
  }
  nm.rebuild_geometry_accels();

  EXPECT_EQ(nm.nonmanifold_edges(), 1u);
  EXPECT_EQ(nm.navcel_neighbors(t0), std::vector<NavCelId>{t3});
  EXPECT_EQ(nm.navcel_neighbors(t3), std::vector<NavCelId>{t0});
  EXPECT_TRUE(nm.navcel_neighbors(t1).empty());
  EXPECT_TRUE(nm.navcel_neighbors(t2).empty());
  expect_symmetric(nm);
}

TEST(NavMap_Adjacency, RebuildClearsStaleNeighbors)
{
  NavMap nm = fan_on_edge(2);
  ASSERT_EQ(nm.navcel_neighbors(0).size(), 1u);
  // Detach the second cell: move it onto its own vertices.
  const auto p = nm.add_vertex({5, 5, 0});
  const auto q = nm.add_vertex({6, 5, 0});
  nm.navcels[1].v[0] = p;
  nm.navcels[1].v[1] = q;
  nm.rebuild_geometry_accels();
  EXPECT_TRUE(nm.navcel_neighbors(0).empty());
  EXPECT_TRUE(nm.navcel_neighbors(1).empty());
}

TEST(NavMap_Adjacency, CountSurvivesCopyAndResetsOnFix)
{
  NavMap nm = fan_on_edge(3);
  const NavMap copy(nm);
  EXPECT_EQ(copy.nonmanifold_edges(), 1u);
  NavMap assigned;
  assigned = nm;
  EXPECT_EQ(assigned.nonmanifold_edges(), 1u);

  // Remove the third cell's use of the edge and rebuild.
  const auto p = nm.add_vertex({9, 9, 0});
  nm.navcels[2].v[0] = p;
  nm.rebuild_geometry_accels();
  EXPECT_EQ(nm.nonmanifold_edges(), 0u);
  expect_symmetric(nm);
}

TEST(NavMap_Adjacency, LargeGridIsManifoldAndSymmetric)
{
  NavMap nm;
  const std::size_t s = nm.create_surface("map");
  const int n = 60;
  for (int j = 0; j <= n; ++j) {
    for (int i = 0; i <= n; ++i) {
      nm.add_vertex({1.0f * i, 1.0f * j, 0.0f});
    }
  }
  auto v = [&](int i, int j) {return static_cast<uint32_t>(j * (n + 1) + i);};
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j), v(i + 1, j + 1)));
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j + 1), v(i, j + 1)));
    }
  }
  nm.rebuild_geometry_accels();
  EXPECT_EQ(nm.nonmanifold_edges(), 0u);
  expect_symmetric(nm);
  std::size_t links = 0;
  for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
    links += nm.navcel_neighbors(cid).size();
  }
  // Interior edges: 3 n^2 - 2 n, each linked from both sides.
  EXPECT_EQ(links, 2u * (3u * n * n - 2u * n));
}
