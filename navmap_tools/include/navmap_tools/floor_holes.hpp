// Copyright 2026 José Miguel Guerrero
//
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

#ifndef NAVMAP_TOOLS__FLOOR_HOLES_HPP_
#define NAVMAP_TOOLS__FLOOR_HOLES_HPP_

#include <vector>

#include "navmap_core/NavMap.hpp"

namespace navmap_tools
{

/// A flat cell of floor: its centre and height
struct FloorCell
{
  float x, y, z;
};

/// The cells (of `cell` m) that close the holes of a NavMap's floors: an empty stretch up to
/// `max_width` m wide between two bits of the same floor, in a row or a column, with the height
/// in between (a ramp across the hole stays a ramp). A straight border does not grow: a bridge
/// needs floor at both ends. Floors are told apart by height, so the hole of the floor below
/// another one is closed too.
std::vector<FloorCell> floor_hole_cells(const ::navmap::NavMap & map, float cell, float max_width);

}  // namespace navmap_tools

#endif  // NAVMAP_TOOLS__FLOOR_HOLES_HPP_
