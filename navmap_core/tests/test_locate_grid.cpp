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

TEST(NavMap_LocateGrid, MultiFloorsChoosesClosestByDz)
{
  NavMap nm;
  // build 2 square floors, z=0 and z=3
  for(int k = 0; k < 2; k++) {
    float z = k * 3.0f;
    int base = nm.positions.x.size();
    nm.positions.x.insert(nm.positions.x.end(), {0, 1, 1, 0});
    nm.positions.y.insert(nm.positions.y.end(), {0, 0, 1, 1});
    nm.positions.z.insert(nm.positions.z.end(), {z, z, z, z});
    nm.navcels.resize(nm.navcels.size() + 2);
    nm.navcels[nm.navcels.size() - 2].v[0] = base + 0;
    nm.navcels[nm.navcels.size() - 2].v[1] = base + 1;
    nm.navcels[nm.navcels.size() - 2].v[2] = base + 2;
    nm.navcels[nm.navcels.size() - 1].v[0] = base + 0;
    nm.navcels[nm.navcels.size() - 1].v[1] = base + 2;
    nm.navcels[nm.navcels.size() - 1].v[2] = base + 3;
    Surface s;
    s.navcels = {(NavCelId)(nm.navcels.size() - 2), (NavCelId)(nm.navcels.size() - 1)};
    nm.surfaces.push_back(s);
  }

  nm.rebuild_geometry_accels();

  Eigen::Vector3f p(0.5f, 0.5f, 2.9f);
  size_t sidx; NavCelId cid; Eigen::Vector3f bary; Eigen::Vector3f hit;
  bool ok = nm.locate_navcel(p, sidx, cid, bary, &hit);
  ASSERT_TRUE(ok);
  EXPECT_EQ(sidx, 1u); // should pick upper floor
  EXPECT_NEAR(hit.z(), 3.0f, 1e-6);
}
