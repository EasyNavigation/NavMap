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

// 06_area_marking: set_area CIRCULAR y RECTANGULAR sobre una malla 1x1 de 2 tris
#include <cmath>
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

  nm.add_layer<uint8_t>("obstacles", "occupancy obstacles", "%", 0);

  // Circular in the center radius 0.3 → marks both centroids
  bool ok1 = nm.set_area<uint8_t>(Vector3f(0.5f, 0.5f, 10.0f), (uint8_t)254,
                                  "obstacles", navmap::AreaShape::CIRCULAR, 0.3f);
  // Rectangular near (0.8,0.2) side 0.35 → mark one
  bool ok2 = nm.set_area<uint8_t>(Vector3f(0.80f, 0.20f, -5.0f), (uint8_t)200,
                                  "obstacles", navmap::AreaShape::RECTANGULAR, 0.35f);

  cout << "set_area circle=" << ok1 << " rect=" << ok2 << endl;
  cout << "c0=" << (int)nm.layer_get<uint8_t>("obstacles", c0,
  0) << " c1=" << (int)nm.layer_get<uint8_t>("obstacles", c1, 0) << endl;
  return 0;
}
