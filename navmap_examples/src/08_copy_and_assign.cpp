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

// 08_copy_and_assign: muestra operator= optimizado (igual geometría) y completo (distinta)
static void fill_one_tri_map(navmap::NavMap & m)
{
  const uint32_t i0 = m.add_vertex({0, 0, 0});
  const uint32_t i1 = m.add_vertex({1, 0, 0});
  const uint32_t i2 = m.add_vertex({0, 1, 0});
  (void)m.add_navcel(i0, i1, i2);
  auto s = m.create_surface("map");
  m.add_navcel_to_surface(s, 0);
  m.add_layer<uint8_t>("occ", "occupancy", "", 0);
  m.add_layer<float>("cost", "traversal cost", "", 0.0f);
  m.layer_set<uint8_t>("occ", 0, 254);
  m.layer_set<float>("cost", 0, 10.0f);
}

int main()
{
  NavMap src; fill_one_tri_map(src);
  NavMap dst; fill_one_tri_map(dst);

  // Same geometry: should not rewrite buffers
  auto names_before = dst.list_layers();
  dst = src;
  auto names_after = dst.list_layers();
  std::cout << "assign equal geom ok; layers after:";
  for(auto & n:names_after) {
    std::cout << " " << n;
  }
  std::cout << "\n";

  // Different geometry in dst → force full copy
  NavMap other;
  auto a = other.add_vertex({0, 0, 0});
  auto b = other.add_vertex({1, 0, 0});
  auto c = other.add_vertex({0, 1, 0});
  auto d = other.add_vertex({1, 1, 0});
  (void)other.add_navcel(a, b, c);
  (void)other.add_navcel(b, d, c);
  other = src;
  std::cout << "other navcels=" << other.navcels.size() << " (esperado 1)\n";
  return 0;
}
