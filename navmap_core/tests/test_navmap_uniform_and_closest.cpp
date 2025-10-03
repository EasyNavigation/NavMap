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
#include <limits>
#include "navmap_core/NavMap.hpp"

using namespace navmap;

namespace
{
constexpr float kEps = 1e-5f;

inline void set_navcel(NavMap & nm, NavCelId cid, PointId a, PointId b, PointId c)
{
  nm.navcels[cid].v[0] = a;
  nm.navcels[cid].v[1] = b;
  nm.navcels[cid].v[2] = c;
}

inline void make_two_floors(NavMap & nm)
{
  nm.positions.x = {0, 1, 0, 1, 0, 1, 0, 1};
  nm.positions.y = {0, 0, 1, 1, 0, 0, 1, 1};
  nm.positions.z = {0, 0, 0, 0, 2, 2, 2, 2};
  nm.navcels.resize(4);
  set_navcel(nm, 0, 0, 1, 2);
  set_navcel(nm, 1, 2, 1, 3);
  set_navcel(nm, 2, 4, 5, 6);
  set_navcel(nm, 3, 6, 5, 7);
  nm.surfaces.resize(2);
  nm.surfaces[0].frame_id = "f0";
  nm.surfaces[0].navcels = {0, 1};
  nm.surfaces[1].frame_id = "f1";
  nm.surfaces[1].navcels = {2, 3};
  nm.rebuild_geometry_accels();
}

inline void make_slope(NavMap & nm)
{
  nm.positions.x = {0, 1, 0, 1};
  nm.positions.y = {0, 0, 1, 1};
  nm.positions.z = {0, 0.2f, 0.4f, 0.6f};
  nm.navcels.resize(2);
  set_navcel(nm, 0, 0, 1, 2);
  set_navcel(nm, 1, 2, 1, 3);
  nm.surfaces.resize(1);
  nm.surfaces[0].frame_id = "terrain";
  nm.surfaces[0].navcels = {0, 1};
  nm.rebuild_geometry_accels();
}
}  // namespace

TEST(NavMap_GridLocate, FlatFloorGridSeedFindsCell) {
  NavMap nm;
  // 1x1 square
  nm.positions.x = {0, 1, 0, 1};
  nm.positions.y = {0, 0, 1, 1};
  nm.positions.z = {0, 0, 0, 0};
  nm.navcels.resize(2);
  set_navcel(nm, 0, 0, 1, 2);
  set_navcel(nm, 1, 2, 1, 3);
  nm.surfaces.resize(1);
  nm.surfaces[0].frame_id = "map";
  nm.surfaces[0].navcels = {0, 1};
  nm.rebuild_geometry_accels();

  // No hint: grid should seed the lookup, no need to raycast in principle.
  Eigen::Vector3f p(0.25f, 0.25f, 0.4f);
  size_t sidx = 999;
  NavCelId cid = 999;
  Eigen::Vector3f bary(0, 0, 0), hit;
  NavMap::LocateOpts opts;
  opts.height_eps = 1.0f;
  ASSERT_TRUE(nm.locate_navcel(p, sidx, cid, bary, &hit, opts));
  EXPECT_EQ(sidx, 0u);
  EXPECT_TRUE(cid == 0 || cid == 1);
  EXPECT_NEAR(hit.z(), 0.0f, kEps);
}

TEST(NavMap_Closest, ChoosesNearestFloorAndPoint) {
  NavMap nm;
  make_two_floors(nm);

  // Query closer to top floor (z=1.7, top at z=2).
  Eigen::Vector3f p(0.5f, 0.5f, 1.7f);
  size_t sidx = 777;
  NavCelId cid = 777;
  Eigen::Vector3f q;
  float dist2 = 0.0f;
  ASSERT_TRUE(nm.closest_navcel(p, sidx, cid, q, dist2));
  EXPECT_EQ(sidx, 1u);
  EXPECT_TRUE(cid == 2 || cid == 3);
  EXPECT_NEAR(q.z(), 2.0f, 1e-3f);
  EXPECT_NEAR(dist2, (2.0f - 1.7f) * (2.0f - 1.7f), 1e-3f);
}

TEST(NavMap_Closest, TerrainDistanceMonotonicity) {
  NavMap nm;
  make_slope(nm);

  Eigen::Vector3f p1(0.25f, 0.25f, 10.0f);
  Eigen::Vector3f q1; float d1;
  size_t s1; NavCelId c1;
  ASSERT_TRUE(nm.closest_navcel(p1, s1, c1, q1, d1));
  EXPECT_GT(d1, 1.0f);

  Eigen::Vector3f p2(0.25f, 0.25f, 1.0f);
  Eigen::Vector3f q2; float d2;
  size_t s2; NavCelId c2;
  ASSERT_TRUE(nm.closest_navcel(p2, s2, c2, q2, d2));
  EXPECT_LT(d2, d1);
}
