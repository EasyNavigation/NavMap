// Copyright 2026 Intelligent Robotics Lab
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

/**
 * @file Descriptors.hpp
 * @brief Geometric descriptors per NavCel (slope, roughness, step, clearance), as layers.
 *
 * They describe the surface only. Turning them into traversability or cost is a decision of the
 * algorithm that uses them (a planner), not of the map.
 */

#ifndef NAVMAP_CORE__DESCRIPTORS_HPP_
#define NAVMAP_CORE__DESCRIPTORS_HPP_

#include <limits>
#include <string>

#include "navmap_core/NavMap.hpp"

namespace navmap
{

/// Which descriptors to compute, and over which neighbourhoods.
struct DescriptorParams
{
  bool slope{true};             ///< "slope": angle of the NavCel normal to the vertical (deg)
  bool roughness{true};         ///< "roughness": RMS distance to the plane fitted in a disc (m)
  bool step{true};              ///< "step": height span of the vertices in a disc (m)
  bool clearance{true};         ///< "clearance": free height to the next surface above (m)
  float roughness_radius{0.3f};  ///< Disc for the roughness plane fit (m)
  float step_radius{0.2f};       ///< Disc for the step (m)
  float clearance_max{10.0f};    ///< Beyond this, no surface above: +inf
};

/**
 * @brief Compute the selected descriptors as F32 layers of @p nm (created or overwritten).
 *
 * Discs grow over the mesh adjacency from each NavCel (navmap/Iterators.hpp), so a descriptor
 * never mixes floors. "clearance" casts a ray up from each centroid against every surface of the
 * map: it is the room up to the next surface the map holds (the floor above), not to a ceiling
 * that is not in the map; +inf when there is none within clearance_max.
 */
void compute_descriptors(NavMap & nm, const DescriptorParams & params = {});

}  // namespace navmap

#endif  // NAVMAP_CORE__DESCRIPTORS_HPP_
