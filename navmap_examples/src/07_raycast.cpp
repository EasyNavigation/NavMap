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

// 07_raycast: simple y batch (raycast_many)
int main()
{
  NavMap nm;
  auto v0 = nm.add_vertex({0, 0, 0});
  auto v1 = nm.add_vertex({1, 0, 0});
  auto v2 = nm.add_vertex({1, 1, 0});
  auto v3 = nm.add_vertex({0, 1, 0});
  auto c0 = nm.add_navcel(v0, v1, v2);
  auto c1 = nm.add_navcel(v0, v2, v3);
  auto s = nm.create_surface("map");
  nm.add_navcel_to_surface(s, c0);
  nm.add_navcel_to_surface(s, c1);
  nm.rebuild_geometry_accels();

  // individual
  navmap::NavCelId cid{}; float t{}; Vector3f hit;
  bool ok = nm.raycast(Vector3f(0.5f, 0.5f, 1.0f), Vector3f(0, 0, -1), cid, t, hit);
  std::cout << "raycast ok=" << ok << " z=" << hit.z() << " cid=" << cid << "\n";

  // many
  std::vector<navmap::Ray> rays(2);
  rays[0] = {Vector3f(0.5f, 0.5f, 1.0f), Vector3f(0, 0, -1)};
  rays[1] = {Vector3f(2.0f, 2.0f, 1.0f), Vector3f(0, 0, -1)};
  std::vector<navmap::RayHit> hits;
  nm.raycast_many(rays, hits, false);
  std::cout << "hits[0].hit=" << hits[0].hit << " z0=" << (hits[0].hit ? hits[0].p.z() : -1) <<
    " hits[1].hit=" << hits[1].hit << "\n";
  return 0;
}
