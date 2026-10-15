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
 * @file Iterators.hpp
 * @brief Traversals over the NavCels of a ::navmap::NavMap: box, disc, ring, polygon, distance
 *        order and segment walk.
 *
 * Every traversal calls a visitor per NavCel. The visitor may return void, or bool: false stops
 * the traversal. Regions around a point (disc, ring, polygon, distance order) grow from a seed
 * NavCel through the mesh adjacency, so they stay on the seed's surface and level: a disc on the
 * ground floor never reaches the floor above, however close it is in XY. The box traversal is
 * global (every surface). A NavCel belongs to a region when its centroid does.
 *
 * Pass a TraversalScratch to reuse buffers across calls: once warm, no traversal allocates.
 */

#ifndef NAVMAP_CORE__ITERATORS_HPP_
#define NAVMAP_CORE__ITERATORS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "navmap_core/NavMap.hpp"

namespace navmap
{

/// Distance from a centre used to grow a region.
enum class Metric
{
  EUCLIDEAN,  ///< 3D distance from the centre to each NavCel centroid
  GEODESIC    ///< Along the surface: straight lines on it, bent only around borders
};

/// Reusable traversal buffers. Not thread-safe: one per thread.
class TraversalScratch
{
public:
  /// Starts a traversal over a map of n NavCels: every NavCel unmarked.
  void begin(std::size_t n)
  {
    if (stamp_.size() < n) {
      stamp_.assign(n, 0);
      dist.resize(n);
      epoch_ = 0;
    }
    if (++epoch_ == 0) {  // wrapped: clear the stamps once
      std::fill(stamp_.begin(), stamp_.end(), 0);
      epoch_ = 1;
    }
    stack.clear();
    heap.clear();
  }

  /// Marks cid; false if it was already marked in this traversal.
  bool mark(NavCelId cid)
  {
    if (stamp_[cid] == epoch_) {return false;}
    stamp_[cid] = epoch_;
    return true;
  }

  bool marked(NavCelId cid) const {return stamp_[cid] == epoch_;}

  /// Dijkstra frontier entry: distance, NavCel, and the pivot it is measured from.
  struct Front
  {
    float d;
    NavCelId cid;
    NavCelId pivot_cel;
    float pivot_len;
    Eigen::Vector3f pivot;
  };

  std::vector<float> dist;      ///< Per NavCel, valid once marked: its settled distance

  std::vector<NavCelId> stack;  ///< BFS frontier
  std::vector<Front> heap;      ///< Dijkstra frontier (min-heap)
  std::vector<int> nodes;       ///< BVH node stack

private:
  std::vector<std::uint32_t> stamp_;
  std::uint32_t epoch_{0};
};

namespace detail
{

// Calls the visitor; true unless it returned false.
template<class F, class ... A>
inline bool visit(F & f, A &&... a)
{
  if constexpr (std::is_void_v<std::invoke_result_t<F &, A...>>) {
    f(std::forward<A>(a)...);
    return true;
  } else {
    return static_cast<bool>(f(std::forward<A>(a)...));
  }
}

inline constexpr NavCelId kNoCel = std::numeric_limits<NavCelId>::max();

inline bool boxes_overlap(const AABB & a, const AABB & b)
{
  return a.min.x() <= b.max.x() && a.max.x() >= b.min.x() &&
         a.min.y() <= b.max.y() && a.max.y() >= b.min.y() &&
         a.min.z() <= b.max.z() && a.max.z() >= b.min.z();
}

inline AABB navcel_box(const NavMap & nm, NavCelId cid)
{
  AABB b;
  for (const auto v : nm.navcels[cid].v) {
    b.expand(nm.positions.at(v));
  }
  return b;
}

// Even-odd rule in XY.
inline bool in_polygon(const std::vector<Eigen::Vector2f> & poly, float x, float y)
{
  bool in = false;
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const auto & a = poly[i];
    const auto & b = poly[j];
    if ((a.y() > y) != (b.y() > y) &&
      x < (b.x() - a.x()) * (y - a.y()) / (b.y() - a.y()) + a.x())
    {
      in = !in;
    }
  }
  return in;
}

inline bool seed_at(const NavMap & nm, const Eigen::Vector3f & p, NavCelId & seed)
{
  std::size_t s = 0;
  Eigen::Vector3f bary, hit;
  return nm.locate_navcel(p, s, seed, bary, &hit);
}

// Segment walk (see for_each_on_segment).
template<class F>
bool for_each_on_segment_impl(
  const NavMap & nm, NavCelId seed, const Eigen::Vector3f & a, const Eigen::Vector3f & b, F && f)
{
  if (seed >= nm.navcels.size()) {return false;}
  const Eigen::Vector2f a2(a.x(), a.y());
  const Eigen::Vector2f d2(b.x() - a.x(), b.y() - a.y());
  auto cross = [](const Eigen::Vector2f & u, const Eigen::Vector2f & v) {
      return u.x() * v.y() - u.y() * v.x();
    };
  constexpr float kEps = 1e-6f;
  NavCelId c = seed, prev = kNoCel;
  float t_in = 0.0f;
  for (std::size_t steps = 0; steps <= nm.navcels.size(); ++steps) {
    const auto & cel = nm.navcels[c];
    Eigen::Vector2f p[3];
    for (int k = 0; k < 3; ++k) {
      const auto v = nm.positions.at(cel.v[k]);
      p[k] = Eigen::Vector2f(v.x(), v.y());
    }
    const float area2 = cross(p[1] - p[0], p[2] - p[0]);
    if (std::abs(area2) < 1e-12f) {return false;}  // vertical in XY: cannot be walked
    const float sgn = area2 > 0.0f ? 1.0f : -1.0f;
    // Leaves through the edge whose inside-test reaches 0 first, beyond t_in.
    float t_exit = std::numeric_limits<float>::infinity();
    int e_exit = -1;
    for (int e = 0; e < 3; ++e) {
      const Eigen::Vector2f edge = p[(e + 1) % 3] - p[e];
      const float ds = sgn * cross(edge, d2);
      if (ds >= 0.0f) {continue;}
      const float s0 = sgn * cross(edge, a2 - p[e]);
      const float te = std::max(-s0 / ds, t_in);
      if (cel.neighbor[e] == prev && te <= t_in + kEps) {continue;}  // back where it came from
      if (te < t_exit) {
        t_exit = te;
        e_exit = e;
      }
    }
    if (e_exit < 0 || t_exit >= 1.0f) {  // b lies in this NavCel
      return visit(f, c, t_in, 1.0f);
    }
    if (t_exit - t_in > kEps || c == seed) {
      if (!visit(f, c, t_in, t_exit)) {return false;}
    }
    const NavCelId next = cel.neighbor[e_exit];
    if (next == kNoCel) {return false;}  // left the surface through a border
    prev = c;
    c = next;
    t_in = t_exit;
  }
  return false;
}

// Grows the region of NavCels at distance in [0, r_out] from the centre, connected to the seed
// through NavCels of the region; visits those at distance >= r_in as f(cid, distance). The seed
// (it holds the centre) is always expanded, and visited regardless of distance if with_seed.
// GEODESIC visits by distance (non-decreasing up to one neighbour step); EUCLIDEAN breadth-first.
template<class F>
std::size_t grow(
  const NavMap & nm, NavCelId seed, const Eigen::Vector3f & center, float r_in, float r_out,
  Metric metric, bool with_seed, TraversalScratch * scratch, F & f)
{
  if (seed >= nm.navcels.size() || !(r_out >= 0.0f)) {return 0;}
  TraversalScratch local;
  TraversalScratch & sc = scratch ? *scratch : local;
  sc.begin(nm.navcels.size());
  std::size_t visited = 0;
  // Visits c if it passes the filter; false if the visitor asked to stop.
  auto emit = [&](NavCelId c, float d) {
      const bool forced = with_seed && c == seed;
      if (!forced && (d < r_in || d > r_out)) {return true;}
      ++visited;
      return visit(f, c, d);
    };

  if (metric == Metric::EUCLIDEAN) {
    sc.mark(seed);
    sc.stack.push_back(seed);
    std::size_t head = 0;  // the stack is used as a FIFO queue
    while (head < sc.stack.size()) {
      const NavCelId c = sc.stack[head++];
      const float d = (nm.navcel_centroid(c) - center).norm();
      if (c != seed && d > r_out) {continue;}
      if (!emit(c, d)) {return visited;}
      for (const NavCelId n : nm.navcels[c].neighbor) {
        if (n != kNoCel && sc.mark(n)) {sc.stack.push_back(n);}
      }
    }
    return visited;
  }

  // GEODESIC: any-angle Dijkstra (Lazy Theta*). Each NavCel is measured in a straight line from
  // a pivot (first the centre). A neighbour is pushed with its parent's pivot; when it is popped,
  // a walk along the line checks that the line stays on the surface, and if it does not, the
  // nearest settled neighbour becomes its pivot. Exact on planes; around borders and holes it
  // bends at centroids, so it can be about one NavCel longer per bend.
  auto cmp = [](const TraversalScratch::Front & a, const TraversalScratch::Front & b) {
      return a.d > b.d;
    };
  auto push = [&](const TraversalScratch::Front & e) {
      sc.heap.push_back(e);
      std::push_heap(sc.heap.begin(), sc.heap.end(), cmp);
    };
  auto visible = [&](NavCelId from, const Eigen::Vector3f & p, const Eigen::Vector3f & q,
    NavCelId target) {
      NavCelId last = kNoCel;
      const bool reached = for_each_on_segment_impl(
        nm, from, p, q, [&](NavCelId x, float, float) {last = x;});
      return reached && last == target;
    };
  push({(nm.navcel_centroid(seed) - center).norm(), seed, seed, 0.0f, center});
  while (!sc.heap.empty()) {
    std::pop_heap(sc.heap.begin(), sc.heap.end(), cmp);
    TraversalScratch::Front u = sc.heap.back();
    sc.heap.pop_back();
    if (sc.marked(u.cid)) {continue;}
    const Eigen::Vector3f pu = nm.navcel_centroid(u.cid);
    if (u.cid != seed && !visible(u.pivot_cel, u.pivot, pu, u.cid)) {
      // Not seen from the inherited pivot: through the best settled neighbour instead
      u.d = std::numeric_limits<float>::infinity();
      for (const NavCelId m : nm.navcels[u.cid].neighbor) {
        if (m == kNoCel || !sc.marked(m)) {continue;}
        const Eigen::Vector3f pm = nm.navcel_centroid(m);
        const float dm = sc.dist[m] + (pu - pm).norm();
        if (dm < u.d) {u = {dm, u.cid, m, sc.dist[m], pm};}
      }
    }
    sc.mark(u.cid);
    sc.dist[u.cid] = u.d;
    if (u.cid != seed && u.d > r_out) {continue;}  // corrected beyond the radius
    if (!emit(u.cid, u.d)) {return visited;}
    for (const NavCelId n : nm.navcels[u.cid].neighbor) {
      if (n == kNoCel || sc.marked(n)) {continue;}
      // Optimistic: n seen from u's pivot (checked when popped)
      const Eigen::Vector3f pn = nm.navcel_centroid(n);
      const float dn = u.pivot_len + (pn - u.pivot).norm();
      if (dn <= r_out) {push({dn, n, u.pivot_cel, u.pivot_len, u.pivot});}
    }
  }
  return visited;
}

}  // namespace detail

/**
 * @brief Visits the NavCels whose bounding box overlaps @p box, on every surface (via the BVH).
 * @param f Visitor `f(NavCelId)`; returning false stops.
 * @return Number of NavCels visited.
 */
template<class F>
std::size_t for_each_in_aabb(
  const NavMap & nm, const AABB & box, F && f, TraversalScratch * scratch = nullptr)
{
  TraversalScratch local;
  TraversalScratch & sc = scratch ? *scratch : local;
  std::size_t visited = 0;
  for (const auto & s : nm.surfaces) {
    if (s.bvh.empty()) {continue;}
    sc.nodes.clear();
    sc.nodes.push_back(0);
    while (!sc.nodes.empty()) {
      const BVHNode & node = s.bvh[sc.nodes.back()];
      sc.nodes.pop_back();
      if (!detail::boxes_overlap(node.box, box)) {continue;}
      if (!node.is_leaf()) {
        sc.nodes.push_back(node.left);
        sc.nodes.push_back(node.right);
        continue;
      }
      for (int i = 0; i < node.count; ++i) {
        const auto cid = static_cast<NavCelId>(s.prim_indices[node.start + i]);
        if (!detail::boxes_overlap(detail::navcel_box(nm, cid), box)) {continue;}
        ++visited;
        if (!detail::visit(f, cid)) {return visited;}
      }
    }
  }
  return visited;
}

/**
 * @brief Visits the NavCels within @p radius of @p center, connected to @p seed (which should
 *        contain the centre). The seed is always visited.
 * @param f Visitor `f(NavCelId)`; returning false stops.
 * @return Number of NavCels visited.
 */
template<class F>
std::size_t for_each_in_disc(
  const NavMap & nm, NavCelId seed, const Eigen::Vector3f & center, float radius, F && f,
  Metric metric = Metric::EUCLIDEAN, TraversalScratch * scratch = nullptr)
{
  auto g = [&](NavCelId c, float) {return detail::visit(f, c);};
  return detail::grow(nm, seed, center, 0.0f, radius, metric, true, scratch, g);
}

/// As above, locating the seed under @p center; visits nothing if the centre is off the map.
template<class F>
std::size_t for_each_in_disc(
  const NavMap & nm, const Eigen::Vector3f & center, float radius, F && f,
  Metric metric = Metric::EUCLIDEAN, TraversalScratch * scratch = nullptr)
{
  NavCelId seed;
  if (!detail::seed_at(nm, center, seed)) {return 0;}
  return for_each_in_disc(nm, seed, center, radius, std::forward<F>(f), metric, scratch);
}

/**
 * @brief Visits the NavCels at distance in [@p r_in, @p r_out] from @p center, connected to
 *        @p seed through NavCels within @p r_out (the inner part is crossed, not visited).
 * @param f Visitor `f(NavCelId)`; returning false stops.
 * @return Number of NavCels visited.
 */
template<class F>
std::size_t for_each_in_ring(
  const NavMap & nm, NavCelId seed, const Eigen::Vector3f & center, float r_in, float r_out,
  F && f, Metric metric = Metric::EUCLIDEAN, TraversalScratch * scratch = nullptr)
{
  auto g = [&](NavCelId c, float) {return detail::visit(f, c);};
  return detail::grow(nm, seed, center, r_in, r_out, metric, false, scratch, g);
}

/**
 * @brief Visits NavCels by geodesic distance from @p center, up to @p max_distance: a spiral
 *        over the surface (nearest free cell, local search...). The seed comes first, always.
 *        The order is non-decreasing up to the distance between neighbour centroids.
 * @param f Visitor `f(NavCelId, float distance)`; returning false stops.
 * @return Number of NavCels visited.
 */
template<class F>
std::size_t for_each_by_distance(
  const NavMap & nm, NavCelId seed, const Eigen::Vector3f & center, float max_distance, F && f,
  TraversalScratch * scratch = nullptr)
{
  return detail::grow(nm, seed, center, 0.0f, max_distance, Metric::GEODESIC, true, scratch, f);
}

/**
 * @brief Visits the NavCels whose centroid lies inside the XY polygon @p poly, connected to
 *        @p seed through NavCels inside it (so only the seed's level). Even-odd rule.
 * @param f Visitor `f(NavCelId)`; returning false stops.
 * @return Number of NavCels visited.
 */
template<class F>
std::size_t for_each_in_polygon(
  const NavMap & nm, NavCelId seed, const std::vector<Eigen::Vector2f> & poly, F && f,
  TraversalScratch * scratch = nullptr)
{
  if (seed >= nm.navcels.size() || poly.size() < 3) {return 0;}
  TraversalScratch local;
  TraversalScratch & sc = scratch ? *scratch : local;
  sc.begin(nm.navcels.size());
  sc.mark(seed);
  sc.stack.push_back(seed);
  std::size_t head = 0, visited = 0;
  while (head < sc.stack.size()) {
    const NavCelId c = sc.stack[head++];
    const Eigen::Vector3f p = nm.navcel_centroid(c);
    const bool in = detail::in_polygon(poly, p.x(), p.y());
    if (!in && c != seed) {continue;}
    if (in) {
      ++visited;
      if (!detail::visit(f, c)) {return visited;}
    }
    for (const NavCelId n : nm.navcels[c].neighbor) {
      if (n != detail::kNoCel && sc.mark(n)) {sc.stack.push_back(n);}
    }
  }
  return visited;
}

/// As above, locating the seed under @p on_surface (a point of the polygon's level).
template<class F>
std::size_t for_each_in_polygon(
  const NavMap & nm, const Eigen::Vector3f & on_surface,
  const std::vector<Eigen::Vector2f> & poly, F && f, TraversalScratch * scratch = nullptr)
{
  NavCelId seed;
  if (!detail::seed_at(nm, on_surface, seed)) {return 0;}
  return for_each_in_polygon(nm, seed, poly, std::forward<F>(f), scratch);
}

/**
 * @brief Walks the surface along segment a -> b (in XY), from @p seed (the NavCel under a),
 *        visiting each NavCel crossed with the segment parameters where it enters and leaves.
 *
 * The walk follows the mesh adjacency, so it stays on the seed's level, also on slopes.
 *
 * @param f Visitor `f(NavCelId, float t_in, float t_out)`, 0 <= t_in <= t_out <= 1; returning
 *          false stops.
 * @return True if the walk reached the NavCel under b; false if it left the surface through a
 *         border first (or was stopped).
 */
template<class F>
bool for_each_on_segment(
  const NavMap & nm, NavCelId seed, const Eigen::Vector3f & a, const Eigen::Vector3f & b, F && f)
{
  return detail::for_each_on_segment_impl(nm, seed, a, b, std::forward<F>(f));
}

/// As above, locating the seed under @p a; false if a is off the map.
template<class F>
bool for_each_on_segment(
  const NavMap & nm, const Eigen::Vector3f & a, const Eigen::Vector3f & b, F && f)
{
  NavCelId seed;
  if (!detail::seed_at(nm, a, seed)) {return false;}
  return for_each_on_segment(nm, seed, a, b, std::forward<F>(f));
}

}  // namespace navmap

#endif  // NAVMAP_CORE__ITERATORS_HPP_
