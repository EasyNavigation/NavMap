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
 * @file Subdivide.hpp
 * @brief Splitting NavCels: along straight lines (cut_along_lines) and around points (refine).
 *
 * Both keep the mesh conforming (every edge is shared whole by the NavCels on both sides: no
 * cracks) and carry what the NavCels hold: each new NavCel has the layer values and the surface
 * of the NavCel it comes from. New vertices interpolate the position and the vertex colours.
 * NavCel ids change; geometry accelerations are rebuilt.
 */

#ifndef NAVMAP_CORE__SUBDIVIDE_HPP_
#define NAVMAP_CORE__SUBDIVIDE_HPP_

#include <cstddef>
#include <functional>
#include <vector>

#include "navmap_core/NavMap.hpp"

namespace navmap
{

/// Sizes before and after a subdivision.
struct SubdivideStats
{
  std::size_t navcels_before{0}, navcels_after{0};
  std::size_t vertices_before{0}, vertices_after{0};
};

/// Lines in the map frame: x = xs[i] and y = ys[j] (m).
struct CutLines
{
  std::vector<float> xs;
  std::vector<float> ys;
};

/**
 * @brief Split the NavCels the lines cross, so that none crosses one.
 *
 * E.g. to cut a map into tiles along a grid. A vertex within @p snap (m) of a line is moved onto
 * it, and the new vertices are put exactly on it: every vertex on a line has exactly the line's
 * coordinate. A link keeps its ends on the first piece of each.
 */
SubdivideStats cut_along_lines(NavMap & nm, const CutLines & lines, float snap = 1e-4f);

/**
 * @brief Refine the NavCels under @p points until none of them has an edge longer than
 *        @p max_edge (m).
 *
 * Longest-edge bisection: an edge is split at its middle in every NavCel that has it, after
 * splitting first, if needed, the longer edges of its neighbours (so the NavCels keep their
 * shape and the mesh stays conforming). The refinement spreads a little around each point.
 * Points off the map are ignored; NavCels at the end of a link are not split, nor any edge they
 * have (the refinement stops short there).
 */
SubdivideStats refine(NavMap & nm, const std::vector<Eigen::Vector3f> & points, float max_edge);

/// The largest edge (m) a NavCel with these vertices may have; not finite or <= 0: no bound.
using EdgeSizeAt = std::function<float(
      const Eigen::Vector3f &, const Eigen::Vector3f &, const Eigen::Vector3f &)>;

/**
 * @brief Refine until no NavCel has an edge longer than @p max_edge_at says for it.
 *
 * The same longest-edge bisection, for a region instead of points (e.g. every NavCel within
 * some distance of an obstacle, or a size that grows with the distance to a robot): the NavCels
 * split are asked again, so the result meets the bound everywhere, in one pass over the map.
 * If no NavCel is too large the map is left as it is (its accelerations too).
 */
SubdivideStats refine(NavMap & nm, const EdgeSizeAt & max_edge_at);

}  // namespace navmap

#endif  // NAVMAP_CORE__SUBDIVIDE_HPP_
