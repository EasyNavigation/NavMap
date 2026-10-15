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
#include <vector>

#include "navmap_core/NavMap.hpp"

using navmap::LinkType;
using navmap::NavCelId;
using navmap::NavMap;

namespace
{

// Two separate 2 x 2 m floors, 3 m apart (8 NavCels each): surfaces 0 and 1.
NavMap two_floors()
{
  NavMap nm;
  for (const float z : {0.0f, 3.0f}) {
    const std::size_t s = nm.create_surface("map");
    const auto base = static_cast<uint32_t>(nm.positions.size());
    for (int j = 0; j <= 2; ++j) {
      for (int i = 0; i <= 2; ++i) {
        nm.add_vertex({static_cast<float>(i), static_cast<float>(j), z});
      }
    }
    auto v = [&](int i, int j) {return base + static_cast<uint32_t>(j * 3 + i);};
    for (int j = 0; j < 2; ++j) {
      for (int i = 0; i < 2; ++i) {
        nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j), v(i + 1, j + 1)));
        nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j + 1), v(i, j + 1)));
      }
    }
  }
  nm.rebuild_geometry_accels();
  return nm;
}

bool has(const std::vector<uint32_t> & v, uint32_t x)
{
  return std::find(v.begin(), v.end(), x) != v.end();
}

}  // namespace

TEST(Links, AddQueryAndDefaultLength)
{
  NavMap nm = two_floors();
  // An elevator from the ground floor (cell 0) to the upper one (cell 8), both ways
  const auto e = nm.add_link(0, 8, LinkType::ELEVATOR);
  ASSERT_NE(e, NavMap::kInvalidLink);
  EXPECT_NEAR(nm.links()[e].length, (nm.navcel_centroid(8) - nm.navcel_centroid(0)).norm(), 1e-6f);
  EXPECT_TRUE(nm.links()[e].bidirectional);
  EXPECT_TRUE(has(nm.links_from(0), e));
  EXPECT_TRUE(has(nm.links_from(8), e));
  // One-way stairs with a given length
  const auto st = nm.add_link(3, 12, LinkType::STAIRS, 4.5f, false);
  EXPECT_FLOAT_EQ(nm.links()[st].length, 4.5f);
  EXPECT_TRUE(has(nm.links_from(3), st));
  EXPECT_FALSE(has(nm.links_from(12), st));
  EXPECT_TRUE(nm.links_from(5).empty());
  EXPECT_TRUE(nm.links_from(99999).empty());  // out of range: no links
  // Links do not change the mesh adjacency
  for (NavCelId n : nm.navcel_neighbors(0)) {
    EXPECT_LT(n, 8u);
                                                              }
}

TEST(Links, InvalidLinksAreRejected)
{
  NavMap nm = two_floors();
  EXPECT_EQ(nm.add_link(0, 99, LinkType::DOOR), NavMap::kInvalidLink);
  EXPECT_EQ(nm.add_link(99, 0, LinkType::DOOR), NavMap::kInvalidLink);
  EXPECT_EQ(nm.add_link(4, 4, LinkType::DOOR), NavMap::kInvalidLink);  // to itself
  EXPECT_TRUE(nm.links().empty());
  EXPECT_FALSE(nm.remove_link(0));
}

TEST(Links, RemoveAndClearKeepTheIndexConsistent)
{
  NavMap nm = two_floors();
  const auto a = nm.add_link(0, 8);
  const auto b = nm.add_link(1, 9);
  const auto c = nm.add_link(2, 10);
  ASSERT_TRUE(has(nm.links_from(1), b));  // builds the index
  EXPECT_TRUE(nm.remove_link(a));          // c takes index a
  ASSERT_EQ(nm.links().size(), 2u);
  EXPECT_TRUE(nm.links_from(0).empty());
  EXPECT_TRUE(has(nm.links_from(2), a));
  EXPECT_EQ(nm.links()[a].from, 2u);
  (void)c;
  nm.clear_links();
  EXPECT_TRUE(nm.links().empty());
  EXPECT_TRUE(nm.links_from(1).empty());
}

TEST(Links, CopiesAndAssignmentsCarryTheLinks)
{
  NavMap src = two_floors();
  src.add_link(0, 8, LinkType::RAMP, 7.0f);
  NavMap copy(src);
  NavMap assigned;
  assigned = src;
  NavMap synced = two_floors();  // same geometry: the layer-sync path of operator=
  synced = src;
  for (const NavMap * m : {&copy, &assigned, &synced}) {
    ASSERT_EQ(m->links().size(), 1u);
    EXPECT_EQ(m->links()[0].type, LinkType::RAMP);
    EXPECT_FLOAT_EQ(m->links()[0].length, 7.0f);
    EXPECT_FALSE(m->links_from(8).empty());
  }
  // Independent afterwards
  copy.clear_links();
  EXPECT_EQ(src.links().size(), 1u);
  NavMap moved(std::move(assigned));
  EXPECT_EQ(moved.links().size(), 1u);
}

TEST(GeometryId, EqualForEqualGeometryAndFollowsEdits)
{
  NavMap a = two_floors(), b = two_floors();
  EXPECT_EQ(a.geometry_id(), b.geometry_id());
  EXPECT_NE(a.geometry_id(), 0u);
  NavMap copy(a);
  EXPECT_EQ(copy.geometry_id(), a.geometry_id());
  // Layers and links are not geometry
  a.add_layer<float>("f", "", "", 1.0f);
  a.add_link(0, 8);
  EXPECT_EQ(a.geometry_id(), b.geometry_id());
  // Editing the geometry changes it (the fingerprint used to go stale after a first query)
  const auto before = b.geometry_id();
  b.add_vertex({9.0f, 9.0f, 9.0f});
  EXPECT_NE(b.geometry_id(), before);
  NavMap c = two_floors();
  const auto c0 = c.geometry_id();
  c.positions.z[0] += 0.1f;
  c.rebuild_geometry_accels();
  EXPECT_NE(c.geometry_id(), c0);
}
