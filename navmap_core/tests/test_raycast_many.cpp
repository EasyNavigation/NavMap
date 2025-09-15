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
#include "navmap_core/NavMap.hpp"

using namespace navmap;

TEST(NavMap_RaycastMany, FlatFloorBatchHits)
{
  NavMap nm;
  // Simple square floor z=0
  nm.positions.x = {0, 1, 1, 0};
  nm.positions.y = {0, 0, 1, 1};
  nm.positions.z = {0, 0, 0, 0};

  nm.navcels.resize(2);
  nm.navcels[0].v[0] = 0; nm.navcels[0].v[1] = 1; nm.navcels[0].v[2] = 2;
  nm.navcels[1].v[0] = 0; nm.navcels[1].v[1] = 2; nm.navcels[1].v[2] = 3;
  nm.surfaces.resize(1);
  nm.surfaces[0].navcels = {0, 1};

  nm.rebuild_geometry_accels();

  std::vector<Ray> rays(2);
  rays[0] = {Eigen::Vector3f(0.5f, 0.5f, 1.0f), Eigen::Vector3f(0, 0, -1)};
  rays[1] = {Eigen::Vector3f(2.0f, 2.0f, 1.0f), Eigen::Vector3f(0, 0, -1)};

  std::vector<RayHit> hits;
  nm.raycast_many(rays, hits, false);

  ASSERT_TRUE(hits[0].hit);
  EXPECT_NEAR(hits[0].p.z(), 0.0f, 1e-6);
  EXPECT_FALSE(hits[1].hit);
}
