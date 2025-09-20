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


#include <iostream>
#include <vector>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <Eigen/Core>

#include "navmap_core/NavMap.hpp"

using navmap::NavMap;
using navmap::NavCelId;
using navmap::Surface;
using navmap::LayerView;
using navmap::LayerType;
using Eigen::Vector3f;
using std::cout; using std::cerr; using std::endl;

// 02_two_floors: two stacked floors, locate & closest_triangle
static void make_two_floors(NavMap & nm, float z0, float z1)
{
  // floor 0
  nm.positions.x = {0, 1, 0, 1};
  nm.positions.y = {0, 0, 1, 1};
  nm.positions.z = {z0, z0, z0, z0};
  nm.navcels.resize(2);
  nm.navcels[0].v[0] = 0; nm.navcels[0].v[1] = 1; nm.navcels[0].v[2] = 2;
  nm.navcels[1].v[0] = 0; nm.navcels[1].v[1] = 2; nm.navcels[1].v[2] = 3;
  nm.surfaces.resize(2);
  nm.surfaces[0].frame_id = "floor_0";
  nm.surfaces[0].navcels = {0, 1};

  // floor 1 (append vertices)
  nm.positions.x.insert(nm.positions.x.end(), {0, 1, 0, 1});
  nm.positions.y.insert(nm.positions.y.end(), {0, 0, 1, 1});
  nm.positions.z.insert(nm.positions.z.end(), {z1, z1, z1, z1});
  nm.navcels.resize(4);
  nm.navcels[2].v[0] = 4; nm.navcels[2].v[1] = 5; nm.navcels[2].v[2] = 6;
  nm.navcels[3].v[0] = 4; nm.navcels[3].v[1] = 6; nm.navcels[3].v[2] = 7;
  nm.surfaces[1].frame_id = "floor_1";
  nm.surfaces[1].navcels = {2, 3};

  nm.rebuild_geometry_accels();
}
int main()
{
  NavMap nm; make_two_floors(nm, 0.0f, 3.0f);

  // locate near top
  Vector3f p(0.5f, 0.5f, 2.8f), hit, bary;
  size_t sidx{}; NavCelId cid{};
  bool ok = nm.locate_navcel(p, sidx, cid, bary, &hit);
  cout << "locate=" << ok << " sidx=" << sidx << " cid=" << cid << " hit.z=" << hit.z() << endl;

  // closest_triangle
  Vector3f q; float d2;
  bool ok2 = nm.closest_triangle(p, sidx, cid, q, d2);
  cout << "closest=" << ok2 << " sidx=" << sidx << " cid=" << cid << " q.z=" << q.z() << " d2=" <<
    d2 << endl;
  return 0;
}
