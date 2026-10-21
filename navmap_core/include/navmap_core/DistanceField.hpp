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
 * @file DistanceField.hpp
 * @brief Incremental distance field over a NavMap, updated by the seed cells added and removed
 *        instead of recomputed: the cost of an update depends on the area it changes.
 */

#ifndef NAVMAP_CORE__DISTANCEFIELD_HPP_
#define NAVMAP_CORE__DISTANCEFIELD_HPP_

#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "navmap_core/NavMap.hpp"

namespace navmap
{

/**
 * @brief Distance from each NavCel to the nearest seed (seed NavCels and, optionally, the open
 *        edges of the mesh), kept up to date incrementally.
 *
 * With a finite max_distance the field is exact for this definition: the distance of a NavCel is
 * the straight 3D distance from its centroid to the nearest seed (point, or open-edge segment)
 * that reaches it through adjacent NavCels all within max_distance of that seed, so it never
 * jumps to another floor or across a hole. Each seed paints its disc; removing one repaints the
 * seeds within 2 * max_distance of it (found through a spatial index). Without max_distance it falls back to NavMap::distance_field() (nearest-feature
 * propagation, approximate) and updates recompute everything.
 *
 * Bound to the geometry of the NavMap it was built from: rebuild it if the geometry changes.
 * The map must outlive it.
 */
class DistanceField
{
public:
  /**
   * @param nm            Map (geometry and adjacency must be built).
   * @param from_boundary Also seed the open edges (walls, drops, unmapped space).
   * @param max_distance  Distances beyond this are +inf (bounds the work of every update).
   */
  DistanceField(
    const NavMap & nm, bool from_boundary,
    float max_distance = std::numeric_limits<float>::infinity());

  /// Full computation from these seed cells.
  void compute(const std::vector<NavCelId> & seed_cells);

  /**
   * @brief Incremental update: NavCels that became seeds and NavCels that stopped being seeds
   *        since the last compute() / update(). Unknown or repeated ids are ignored.
   */
  void update(const std::vector<NavCelId> & added, const std::vector<NavCelId> & removed);

  const std::vector<float> & distances() const {return dist_;}
  float operator[](NavCelId cid) const {return dist_[cid];}

  /// NavCels whose distance changed in the last compute() / update().
  const std::vector<NavCelId> & changed() const {return changed_;}

  /// Whether @p cid is currently a seed cell.
  bool is_seed(NavCelId cid) const {return cid < seed_of_.size() && seed_of_[cid] != kNoSeed;}

private:
  static constexpr std::uint32_t kNoSeed = std::numeric_limits<std::uint32_t>::max();
  struct Seed
  {
    Eigen::Vector3f a, b;  ///< point (a == b) or segment
    NavCelId cell;         ///< the NavCel it starts from
    bool alive;
  };

  float distance_to(NavCelId c, std::uint32_t seed) const;
  void touch(NavCelId c);
  void begin_operation();
  void finish();
  template<class F>
  void flood(std::uint32_t seed, F && f);
  void splat(std::uint32_t seed);
  std::uint32_t new_point_seed(NavCelId c);
  static std::int64_t bucket_key(int bx, int by, int bz);
  void index_seed(std::uint32_t seed, bool insert);

  const NavMap & nm_;
  bool from_boundary_;
  float max_distance_;
  float bucket_{1.0f};
  std::unordered_map<std::int64_t, std::vector<std::uint32_t>> buckets_;  ///< seed index
  std::vector<Eigen::Vector3f> centroid_;
  std::vector<Seed> seeds_;                    ///< open-edge seeds first, then point seeds
  std::size_t n_boundary_{0};
  std::vector<std::uint32_t> boundary_start_;  ///< per cell: its open-edge seeds (CSR)
  std::vector<std::uint32_t> seed_of_;         ///< per cell: its point seed, if a seed cell
  std::vector<std::uint32_t> nearest_;         ///< per cell: the seed its distance comes from
  std::vector<float> dist_;
  std::vector<std::uint32_t> free_seed_slots_;
  // Cells touched by the current operation and their previous distance
  std::vector<NavCelId> touched_;
  std::vector<float> before_;
  std::vector<std::uint32_t> touched_stamp_;
  std::uint32_t stamp_{0};
  std::vector<NavCelId> changed_;
  // Flood buffers
  std::vector<NavCelId> flood_;
  std::vector<std::uint32_t> flood_stamp_;
  std::uint32_t flood_id_{0};
};

}  // namespace navmap

#endif  // NAVMAP_CORE__DISTANCEFIELD_HPP_
