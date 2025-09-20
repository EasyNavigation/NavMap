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
