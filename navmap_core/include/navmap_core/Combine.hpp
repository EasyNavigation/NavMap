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
 * @file Combine.hpp
 * @brief Combining one layer into another with explicit operators, unknown-aware, with provenance.
 *
 * Each source (a static prior, obstacles from a sensor, a keep-out zone) keeps its own layer, and
 * a master layer is composed from them. Clearing a source layer then never erases another source:
 * a prior survives sensor clearing because it is never written by the sensor.
 *
 * The cost convention (Costs.hpp) is that of costmap_2d / Nav2.
 */

#ifndef NAVMAP_CORE__COMBINE_HPP_
#define NAVMAP_CORE__COMBINE_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "navmap_core/NavMap.hpp"

namespace navmap
{

/// How a source value s is combined into the destination value d (as costmap_2d's update modes).
enum class CombineOp : uint8_t
{
  OVERWRITE,         ///< d = s, except where s is unknown
  TRUE_OVERWRITE,    ///< d = s, unknown included
  MAX,               ///< d = max(d, s); s replaces an unknown d; an unknown s is ignored
  MAX_KEEP_UNKNOWN,  ///< as MAX, but an unknown d stays unknown
  MIN,               ///< d = min(d, s); s replaces an unknown d; an unknown s is ignored
  ADD,               ///< d = min(d + s, cap); s replaces an unknown d; an unknown s is ignored
};

/// Options of combine_layer().
struct CombineOptions
{
  /// Cells to combine (e.g. the source's changed()); all cells if empty.
  std::vector<NavCelId> cells;
  /// ADD saturates here (and at the range of the destination type). For a cost layer use
  /// INSCRIBED_INFLATED_OBSTACLE, as costmap_2d does, so that a sum is never read as lethal.
  double cap{std::numeric_limits<double>::infinity()};
  /// U8 layer recording which source set each cell (created if absent, 0 = none). Empty: no record.
  std::string provenance;
  /// Value written to the provenance layer for this source.
  std::uint8_t source_id{1};
};

/**
 * @brief Combine layer @p src into layer @p dst cell by cell with @p op.
 *
 * Both layers must exist with one value per NavCel; their types may differ (values convert as in
 * NavMap::layer_get: integers are rounded and clamped to their range). Which value is unknown is
 * each layer's own LayerMeta::unknown (NaN allowed). Writing an unknown value into @p dst writes
 * the unknown of @p dst; if @p dst has none, nothing is written.
 *
 * Cells whose value changes are recorded with mark_changed() in @p dst. The provenance layer, if
 * requested, gets source_id wherever the result is the source's value (MAX/MIN: it won or tied;
 * ADD: it added something), even if the value was already the same.
 *
 * @return Cells whose @p dst value changed, or -1 if a layer is missing, sizes differ or
 *         @p dst == @p src.
 */
std::ptrdiff_t combine_layer(
  NavMap & nm, const std::string & dst, const std::string & src, CombineOp op,
  const CombineOptions & options = {});

}  // namespace navmap

#endif  // NAVMAP_CORE__COMBINE_HPP_
