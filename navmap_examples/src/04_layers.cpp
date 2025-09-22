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

// 04_layers: add/list/set/get
int main()
{
  NavMap nm;
  // Minimal geometry: one tri
  auto v0 = nm.add_vertex({0, 0, 0});
  auto v1 = nm.add_vertex({1, 0, 0});
  auto v2 = nm.add_vertex({0, 1, 0});
  auto c0 = nm.add_navcel(v0, v1, v2);
  auto s = nm.create_surface("map");
  nm.add_navcel_to_surface(s, c0);
  nm.rebuild_geometry_accels();

  nm.add_layer<uint8_t>("occ", "occupancy", "%", 0);
  nm.add_layer<float>("cost", "cost", "", 0.0f);
  nm.layer_set<uint8_t>("occ", c0, 254);
  nm.layer_set<float>("cost", c0, 5.5f);

  auto names = nm.list_layers();
  cout << "Layers:"; for(auto & n:names) {
    cout << " " << n;
  }
  cout << endl;
  cout << "occ=" << (int)nm.layer_get<uint8_t>("occ", c0,
    0) << ", cost=" << nm.layer_get<double>("cost", c0, -1.0) << endl;
  return 0;
}
