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
