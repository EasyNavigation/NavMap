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

#include "navmap_core/DistanceField.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "navmap_core/Geometry.hpp"

namespace navmap
{

namespace
{
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr NavCelId kNone = std::numeric_limits<NavCelId>::max();
}  // namespace

DistanceField::DistanceField(const NavMap & nm, bool from_boundary, float max_distance)
: nm_(nm), from_boundary_(from_boundary), max_distance_(max_distance)
{
  const std::size_t n = nm_.navcels.size();
  centroid_.resize(n);
  for (NavCelId c = 0; c < n; ++c) {
    centroid_[c] = nm_.navcel_centroid(c);
  }
  seed_of_.assign(n, kNoSeed);
  nearest_.assign(n, kNoSeed);
  dist_.assign(n, kInf);
  touched_stamp_.assign(n, 0);
  flood_stamp_.assign(n, 0);
  before_.assign(n, kInf);
  // Open edges: fixed segment seeds, indexed by their cell (CSR)
  boundary_start_.assign(n + 1, 0);
  if (from_boundary_) {
    for (NavCelId c = 0; c < n; ++c) {
      for (int e = 0; e < 3; ++e) {
        if (nm_.navcels[c].neighbor[e] != kNone) {continue;}
        seeds_.push_back(
          {nm_.positions.at(nm_.navcels[c].v[e]),
            nm_.positions.at(nm_.navcels[c].v[(e + 1) % 3]), c, true});
      }
      boundary_start_[c + 1] = static_cast<std::uint32_t>(seeds_.size());
    }
  }
  n_boundary_ = seeds_.size();
  bucket_ = std::isfinite(max_distance_) && max_distance_ > 0.0f ? max_distance_ : 1.0f;
}

float DistanceField::distance_to(NavCelId c, std::uint32_t seed) const
{
  return point_segment_distance(centroid_[c], seeds_[seed].a, seeds_[seed].b);
}

void DistanceField::touch(NavCelId c)
{
  if (touched_stamp_[c] == stamp_) {return;}
  touched_stamp_[c] = stamp_;
  before_[c] = dist_[c];
  touched_.push_back(c);
}

void DistanceField::begin_operation()
{
  if (++stamp_ == 0) {
    std::fill(touched_stamp_.begin(), touched_stamp_.end(), 0);
    stamp_ = 1;
  }
}

void DistanceField::finish()
{
  changed_.clear();
  for (const NavCelId c : touched_) {
    if (!(before_[c] == dist_[c])) {changed_.push_back(c);}  // inf == inf: unchanged
  }
  touched_.clear();
}

// Floods from the seed's cell over adjacency, calling f(cell, distance) on each cell it enters;
// f returns whether to expand from that cell (the origin always expands).
template<class F>
void DistanceField::flood(std::uint32_t s, F && f)
{
  if (++flood_id_ == 0) {
    std::fill(flood_stamp_.begin(), flood_stamp_.end(), 0);
    flood_id_ = 1;
  }
  const std::size_t n = nm_.navcels.size();
  const NavCelId origin = seeds_[s].cell;
  flood_.clear();
  flood_.push_back(origin);
  flood_stamp_[origin] = flood_id_;
  while (!flood_.empty()) {
    const NavCelId c = flood_.back();
    flood_.pop_back();
    if (!f(c, distance_to(c, s)) && c != origin) {continue;}
    for (const NavCelId v : nm_.navcels[c].neighbor) {
      if (v == kNone || v >= n || flood_stamp_[v] == flood_id_) {continue;}
      flood_stamp_[v] = flood_id_;
      flood_.push_back(v);
    }
  }
}

// Paints seed s over its whole disc (the cells it reaches through adjacency within max_distance),
// where it is nearer. Exact by construction: no pruning (regions are not star-shaped around holes).
void DistanceField::splat(std::uint32_t s)
{
  flood(
    s, [&](NavCelId c, float d) {
      if (d > max_distance_) {return false;}
      if (d < dist_[c]) {
        touch(c);
        dist_[c] = d;
        nearest_[c] = s;
      }
      return true;
    });
}

std::int64_t DistanceField::bucket_key(int bx, int by, int bz)
{
  return (static_cast<std::int64_t>(bx) * 73856093) ^ (static_cast<std::int64_t>(by) * 19349663) ^
         (static_cast<std::int64_t>(bz) * 83492791);
}

void DistanceField::index_seed(std::uint32_t s, bool insert)
{
  const Eigen::Vector3f lo = seeds_[s].a.cwiseMin(seeds_[s].b) / bucket_;
  const Eigen::Vector3f hi = seeds_[s].a.cwiseMax(seeds_[s].b) / bucket_;
  for (int bx = static_cast<int>(std::floor(lo.x())); bx <= static_cast<int>(std::floor(hi.x()));
    ++bx)
  {
    for (int by = static_cast<int>(std::floor(lo.y())); by <= static_cast<int>(std::floor(hi.y()));
      ++by)
    {
      for (int bz =
        static_cast<int>(std::floor(lo.z())); bz <= static_cast<int>(std::floor(hi.z()));
        ++bz)
      {
        auto & v = buckets_[bucket_key(bx, by, bz)];
        if (insert) {
          v.push_back(s);
        } else {
          const auto it = std::find(v.begin(), v.end(), s);
          if (it != v.end()) {*it = v.back(); v.pop_back();}
        }
      }
    }
  }
}

std::uint32_t DistanceField::new_point_seed(NavCelId c)
{
  std::uint32_t s;
  if (!free_seed_slots_.empty()) {
    s = free_seed_slots_.back();
    free_seed_slots_.pop_back();
    seeds_[s] = {centroid_[c], centroid_[c], c, true};
  } else {
    seeds_.push_back({centroid_[c], centroid_[c], c, true});
    s = static_cast<std::uint32_t>(seeds_.size() - 1);
  }
  seed_of_[c] = s;
  index_seed(s, true);
  return s;
}

void DistanceField::compute(const std::vector<NavCelId> & seed_cells)
{
  const std::size_t n = nm_.navcels.size();
  begin_operation();
  for (NavCelId c = 0; c < n; ++c) {
    touch(c);
  }
  std::fill(seed_of_.begin(), seed_of_.end(), kNoSeed);
  seeds_.resize(n_boundary_);
  free_seed_slots_.clear();
  buckets_.clear();
  if (std::isfinite(max_distance_)) {
    for (std::uint32_t b = 0; b < n_boundary_; ++b) {
      index_seed(b, true);
    }
  }
  std::vector<NavCelId> unique;
  unique.reserve(seed_cells.size());
  for (const NavCelId c : seed_cells) {
    if (c < n && seed_of_[c] == kNoSeed) {
      new_point_seed(c);
      unique.push_back(c);
    }
  }
  if (!std::isfinite(max_distance_)) {
    // Unbounded: nearest-feature propagation (NavMap::distance_field), approximate
    dist_ = nm_.distance_field(unique, from_boundary_, max_distance_);
    std::fill(nearest_.begin(), nearest_.end(), kNoSeed);
  } else {
    std::fill(dist_.begin(), dist_.end(), kInf);
    std::fill(nearest_.begin(), nearest_.end(), kNoSeed);
    for (std::uint32_t s = 0; s < seeds_.size(); ++s) {
      splat(s);
    }
  }
  finish();
}

void DistanceField::update(
  const std::vector<NavCelId> & added, const std::vector<NavCelId> & removed)
{
  const std::size_t n = nm_.navcels.size();
  if (!std::isfinite(max_distance_)) {
    // Unbounded: no local update, recompute with the new seed set
    std::vector<NavCelId> seeds;
    for (NavCelId c = 0; c < n; ++c) {
      if (seed_of_[c] != kNoSeed) {seeds.push_back(c);}
    }
    for (const NavCelId c : removed) {
      if (c < n) {std::erase(seeds, c);}
    }
    for (const NavCelId c : added) {
      if (c < n && std::find(seeds.begin(), seeds.end(), c) == seeds.end()) {seeds.push_back(c);}
    }
    const std::vector<float> prev = dist_;
    compute(seeds);
    changed_.clear();
    for (NavCelId c = 0; c < n; ++c) {
      if (!(prev[c] == dist_[c])) {changed_.push_back(c);}
    }
    return;
  }

  begin_operation();
  // 1) Removed seeds: the cells measured to them go back to +inf, and every seed that may cover
  //    one of them is painted again. Such a seed is within max_distance of that cell, so within
  //    2 * max_distance of the removed seed in a straight line: the seed index finds them.
  std::vector<std::uint32_t> repaint;
  for (const NavCelId c : removed) {
    if (c >= n || seed_of_[c] == kNoSeed) {continue;}
    const std::uint32_t s = seed_of_[c];
    seed_of_[c] = kNoSeed;
    seeds_[s].alive = false;
    index_seed(s, false);
    flood(
      s, [&](NavCelId x, float d) {
        if (d > max_distance_) {return false;}
        if (nearest_[x] == s) {
          touch(x);
          dist_[x] = kInf;
          nearest_[x] = kNoSeed;
        }
        return true;
      });
    const Eigen::Vector3f p = seeds_[s].a;
    const float r2 = 2.0f * max_distance_;
    const Eigen::Vector3f lo = (p.array() - r2).matrix() / bucket_;
    const Eigen::Vector3f hi = (p.array() + r2).matrix() / bucket_;
    for (int bx = static_cast<int>(std::floor(lo.x())); bx <= static_cast<int>(std::floor(hi.x()));
      ++bx)
    {
      for (int by =
        static_cast<int>(std::floor(lo.y())); by <= static_cast<int>(std::floor(hi.y()));
        ++by)
      {
        for (int bz = static_cast<int>(std::floor(lo.z()));
          bz <= static_cast<int>(std::floor(hi.z())); ++bz)
        {
          const auto it = buckets_.find(bucket_key(bx, by, bz));
          if (it == buckets_.end()) {continue;}
          for (const std::uint32_t q : it->second) {
            if (point_segment_distance(p, seeds_[q].a, seeds_[q].b) <= r2) {repaint.push_back(q);}
          }
        }
      }
    }
    free_seed_slots_.push_back(s);
  }
  std::sort(repaint.begin(), repaint.end());
  repaint.erase(std::unique(repaint.begin(), repaint.end()), repaint.end());
  for (const std::uint32_t s : repaint) {
    if (seeds_[s].alive) {splat(s);}
  }
  // 2) New seeds
  for (const NavCelId c : added) {
    if (c >= n || seed_of_[c] != kNoSeed) {continue;}
    splat(new_point_seed(c));
  }
  finish();
}

}  // namespace navmap
