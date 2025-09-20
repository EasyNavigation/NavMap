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
