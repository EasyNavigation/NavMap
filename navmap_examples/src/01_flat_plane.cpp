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

// 01_flat_plane: unit square at z=0, 2 triangles, U8 occupancy
static void make_flat_square(NavMap & nm)
{
  nm.positions.x = {0.0f, 1.0f, 0.0f, 1.0f};
  nm.positions.y = {0.0f, 0.0f, 1.0f, 1.0f};
  nm.positions.z = {0.0f, 0.0f, 0.0f, 0.0f};

  nm.navcels.resize(2);
  nm.navcels[0].v[0] = 0; nm.navcels[0].v[1] = 1; nm.navcels[0].v[2] = 2;
  nm.navcels[1].v[0] = 0; nm.navcels[1].v[1] = 2; nm.navcels[1].v[2] = 3;

  nm.surfaces.resize(1);
  nm.surfaces[0].frame_id = "map";
  nm.surfaces[0].navcels = {0, 1};

  nm.rebuild_geometry_accels();
}

int main()
{
  NavMap nm; make_flat_square(nm);

  auto occ = nm.layers.add_or_get<uint8_t>("occupancy", nm.navcels.size(), LayerType::U8);
  if(!occ) {cerr << "Cannot create 'occupancy'\n"; return 1;}
  (*occ)[0] = 0; (*occ)[1] = 254;

  Vector3f p(0.75f, 0.75f, 0.4f);
  size_t sidx{}; NavCelId cid{}; Vector3f bary, hit;
  bool ok = nm.locate_navcel(p, sidx, cid, bary, &hit);
  cout << "locate=" << ok << " sidx=" << sidx << " cid=" << cid << " hit=(" << hit.x() << "," <<
    hit.y() << "," << hit.z() << ")\n";
  if(ok) {
    cout << "occ at cid: " << (int)nm.navcel_value<uint8_t>(cid, *occ) << endl;
  }
  return 0;
}
