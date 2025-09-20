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

// 03_slope_surface: sloped z, sample_layer_at
int main()
{
  NavMap nm;
  nm.positions.x = {0, 1, 0, 1};
  nm.positions.y = {0, 0, 1, 1};
  nm.positions.z = {0, 0.2f, 0.4f, 0.6f};
  nm.navcels.resize(2);
  nm.navcels[0].v[0] = 0; nm.navcels[0].v[1] = 1; nm.navcels[0].v[2] = 2;
  nm.navcels[1].v[0] = 0; nm.navcels[1].v[1] = 2; nm.navcels[1].v[2] = 3;
  nm.surfaces.resize(1);
  nm.surfaces[0].frame_id = "terrain";
  nm.surfaces[0].navcels = {0, 1};
  nm.rebuild_geometry_accels();

  nm.add_layer<float>("traversability", "slope traversability", "");
  nm.layer_set<float>("traversability", 0, 0.2f);
  nm.layer_set<float>("traversability", 1, 0.9f);

  double vA = nm.sample_layer_at("traversability", Vector3f(0.2f, 0.2f, 1.0f), -1.0);
  double vB = nm.sample_layer_at("traversability", Vector3f(0.8f, 0.8f, 1.0f), -1.0);
  cout << "sample A=" << vA << " B=" << vB << endl;
  return 0;
}
