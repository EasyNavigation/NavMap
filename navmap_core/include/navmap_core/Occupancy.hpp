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
 * @file Occupancy.hpp
 * @brief Probabilistic occupancy per NavCel: log-odds with clamping (as OctoMap), stored in an
 *        int16 layer in fixed point, and a three-state classification (as UFOMap).
 *
 * This is the update rule only. Which NavCels a measurement hits or traverses (the sensor
 * model) is decided by the algorithm that uses it.
 */

#ifndef NAVMAP_CORE__OCCUPANCY_HPP_
#define NAVMAP_CORE__OCCUPANCY_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "navmap_core/NavMap.hpp"

namespace navmap
{

/// Probabilities of the update rule and of the classification (OctoMap defaults).
struct OccupancyParams
{
  float p_hit{0.7f};        ///< A measurement ends in the NavCel
  float p_miss{0.4f};       ///< A measurement crosses the NavCel
  float p_min{0.12f};       ///< Clamping: never more certain of free than this
  float p_max{0.97f};       ///< Clamping: never more certain of occupied than this
  float p_occupied{0.6f};   ///< At or above: OCCUPIED
  float p_free{0.4f};       ///< At or below: FREE (in between: UNKNOWN)
};

/// Three-state classification of a NavCel.
enum class OccupancyState : uint8_t { UNKNOWN = 0, FREE = 1, OCCUPIED = 2 };

/**
 * @brief Log-odds occupancy over an int16 layer of a NavMap.
 *
 * Values are log-odds in units of 1/kScale; 0 (probability 0.5) is "never observed" and is
 * declared as the layer's unknown value. Updates add the hit/miss log-odds and clamp.
 */
class OccupancyLayer
{
public:
  static constexpr float kScale = 1000.0f;  ///< int16 units per log-odds unit

  static int16_t to_fixed(float logodds)
  {
    const float v = std::round(logodds * kScale);
    return static_cast<int16_t>(std::clamp(v, -32767.0f, 32767.0f));
  }
  static float logodds_of(float p) {return std::log(p / (1.0f - p));}
  static float probability_of(float logodds) {return 1.0f / (1.0f + std::exp(-logodds));}

  /// Uses (or creates, all unknown) the int16 layer @p name of @p nm.
  explicit OccupancyLayer(
    NavMap & nm, const std::string & name = "occupancy_logodds", OccupancyParams p = {})
  : name_(name),
    hit_(to_fixed(logodds_of(p.p_hit))), miss_(to_fixed(logodds_of(p.p_miss))),
    min_(to_fixed(logodds_of(p.p_min))), max_(to_fixed(logodds_of(p.p_max))),
    occ_(to_fixed(logodds_of(p.p_occupied))), free_(to_fixed(logodds_of(p.p_free)))
  {
    const bool existed = nm.has_layer(name);
    view_ = nm.add_layer<int16_t>(name, "occupancy log-odds", "1/1000 log-odds", 0);
    if (!existed) {nm.set_layer_unknown(name, 0.0);}
  }

  /// A measurement ended in @p cid (occupied evidence).
  void hit(NavCelId cid) {add(cid, hit_);}
  /// A measurement crossed @p cid (free evidence).
  void miss(NavCelId cid) {add(cid, miss_);}
  void update(NavCelId cid, bool occupied) {add(cid, occupied ? hit_ : miss_);}

  /// Adds @p delta (fixed point) to @p cid and clamps to [p_min, p_max].
  void add(NavCelId cid, int16_t delta)
  {
    auto & d = view_->data();
    if (cid >= d.size()) {return;}
    const int v = static_cast<int>(d[cid]) + delta;
    d[cid] = static_cast<int16_t>(std::clamp(v, static_cast<int>(min_), static_cast<int>(max_)));
    view_->mark_dirty();
  }

  /// Forgets @p cid (back to unknown).
  void reset(NavCelId cid)
  {
    auto & d = view_->data();
    if (cid < d.size()) {d[cid] = 0; view_->mark_dirty();}
  }

  float logodds(NavCelId cid) const {return value(cid) / kScale;}
  float probability(NavCelId cid) const {return probability_of(logodds(cid));}

  OccupancyState state(NavCelId cid) const
  {
    const int16_t v = value(cid);
    if (v >= occ_) {return OccupancyState::OCCUPIED;}
    if (v <= free_) {return OccupancyState::FREE;}
    return OccupancyState::UNKNOWN;
  }

  const std::string & name() const {return name_;}
  const std::shared_ptr<LayerView<int16_t>> & view() const {return view_;}

private:
  int16_t value(NavCelId cid) const
  {
    const auto & d = view_->data();
    return cid < d.size() ? d[cid] : int16_t{0};
  }

  std::string name_;
  std::shared_ptr<LayerView<int16_t>> view_;
  int16_t hit_, miss_, min_, max_, occ_, free_;
};

}  // namespace navmap

#endif  // NAVMAP_CORE__OCCUPANCY_HPP_
