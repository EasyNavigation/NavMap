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
 * @file Simplify.hpp
 * @brief Mesh simplification that keeps the layers: large NavCels where surface and layers are
 *        uniform, the original ones where they change.
 */

#ifndef NAVMAP_CORE__SIMPLIFY_HPP_
#define NAVMAP_CORE__SIMPLIFY_HPP_

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "navmap_core/NavMap.hpp"

namespace navmap
{

/// Parameters of simplify().
struct SimplifyParams
{
  /// Geometric error (m): every vertex stays within this distance of the planes of the original
  /// NavCels it replaces (quadric error), and of the open edges it moves along.
  float max_error{0.02f};
  /// No edge longer than this (m). Bounds the NavCel size, e.g. to keep marking obstacles fine.
  float max_edge_length{std::numeric_limits<float>::infinity()};
  /// Minimum NavCel quality, 4·sqrt(3)·area / sum of squared edges (1 equilateral, 0 flat).
  float min_quality{0.15f};
  /// Maximum rotation (deg) of a NavCel's normal with respect to the original one.
  float max_normal_change{30.0f};
  /// Layers whose values must not change anywhere; empty: all the layers of the map.
  std::vector<std::string> layers;
  /// Layers left out of the above (e.g. per-cell geometry such as "elevation"). They are carried
  /// from the NavCel that survives, so recompute them if they depend on its shape.
  std::vector<std::string> ignore_layers;
};

/// Sizes before and after simplify().
struct SimplifyStats
{
  std::size_t navcels_before{0}, navcels_after{0};
  std::size_t vertices_before{0}, vertices_after{0};
};

/**
 * @brief Simplify @p nm in place by half-edge collapses, cheapest first.
 *
 * A vertex is removed only if all its NavCels have the same values in the kept layers and belong to
 * the same surface, so every layer keeps its value at every point of the map: borders between
 * values (an obstacle, unknown space, a different cost) are kept exactly. Open edges keep their
 * shape (within max_error), topology is preserved (no folds, no new holes or bridges), and NavCels
 * that are the end of a link survive. Layers, surfaces, links and vertex colors are carried over;
 * NavCel and vertex ids change. Geometry accelerations are rebuilt.
 */
SimplifyStats simplify(NavMap & nm, const SimplifyParams & params = {});

}  // namespace navmap

#endif  // NAVMAP_CORE__SIMPLIFY_HPP_
