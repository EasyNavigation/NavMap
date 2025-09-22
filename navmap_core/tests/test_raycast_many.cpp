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
