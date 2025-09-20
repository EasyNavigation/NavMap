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

// 05_neighbors_and_centroids
static void make_flat_square(NavMap & nm)
{
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
}
int main()
{
  NavMap nm; make_flat_square(nm);
  auto c0 = nm.surfaces[0].navcels[0];
  auto c1 = nm.surfaces[0].navcels[1];
  auto cc0 = nm.navcel_centroid(c0);
  auto cc1 = nm.navcel_centroid(c1);
  auto neigh = nm.navcel_neighbors(c0);
  cout << "centroid0=(" << cc0.x() << "," << cc0.y() << "," << cc0.z() << ")" << endl;
  cout << "centroid1=(" << cc1.x() << "," << cc1.y() << "," << cc1.z() << ")" << endl;
  cout << "neighbors of c0:"; for(auto n:neigh) {
    cout << " " << n;
  }
  cout << endl;
  return 0;
}
