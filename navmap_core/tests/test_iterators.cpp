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

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include "navmap_core/Iterators.hpp"
#include "navmap_core/NavMap.hpp"

using navmap::AABB;
using navmap::Metric;
using navmap::NavCelId;
using navmap::NavMap;
using navmap::TraversalScratch;

namespace
{

constexpr NavCelId kNone = std::numeric_limits<NavCelId>::max();

// nx x ny cells of `res` m at z = f(x, y) (+ origin), skipping cells where skip(i, j).
std::size_t add_grid(
  NavMap & nm, int nx, int ny, float res, const std::function<float(float, float)> & f,
  const std::function<bool(int, int)> & skip = nullptr, float ox = 0.0f, float oy = 0.0f)
{
  const std::size_t s = nm.create_surface("map");
  const auto base = static_cast<uint32_t>(nm.positions.size());
  for (int j = 0; j <= ny; ++j) {
    for (int i = 0; i <= nx; ++i) {
      const float x = ox + i * res, y = oy + j * res;
      nm.add_vertex({x, y, f(x, y)});
    }
  }
  auto v = [&](int i, int j) {return base + static_cast<uint32_t>(j * (nx + 1) + i);};
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      if (skip && skip(i, j)) {continue;}
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j), v(i + 1, j + 1)));
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j + 1), v(i, j + 1)));
    }
  }
  return s;
}

auto flat(float h) {return [h](float, float) {return h;};}

NavMap flat_map(int n = 20, float res = 0.5f)
{
  NavMap nm;
  add_grid(nm, n, n, res, flat(0.0f));
  nm.rebuild_geometry_accels();
  return nm;
}

// Two floors over the same 10 x 10 m footprint, 3 m apart, not connected.
NavMap two_floors()
{
  NavMap nm;
  add_grid(nm, 20, 20, 0.5f, flat(0.0f));
  add_grid(nm, 20, 20, 0.5f, flat(3.0f));
  nm.rebuild_geometry_accels();
  return nm;
}

NavCelId seed_at(const NavMap & nm, const Eigen::Vector3f & p)
{
  std::size_t s; NavCelId c; Eigen::Vector3f bary, hit;
  EXPECT_TRUE(nm.locate_navcel(p, s, c, bary, &hit)) << p.transpose();
  return c;
}

template<class Range>
std::set<NavCelId> collect(Range && run)
{
  std::set<NavCelId> out;
  std::size_t calls = 0;
  const std::size_t n = run([&](NavCelId c) {++calls; EXPECT_TRUE(out.insert(c).second) << c;});
  EXPECT_EQ(n, calls);
  return out;
}

// NavCels reachable from seed through NavCels satisfying `in` (the seed always expands).
std::set<NavCelId> flood(const NavMap & nm, NavCelId seed, const std::function<bool(NavCelId)> & in)
{
  std::set<NavCelId> seen{seed}, out;
  std::vector<NavCelId> st{seed};
  while (!st.empty()) {
    const NavCelId c = st.back();
    st.pop_back();
    if (c != seed && !in(c)) {continue;}
    if (in(c)) {out.insert(c);}
    for (NavCelId n : nm.navcels[c].neighbor) {
      if (n != kNone && seen.insert(n).second) {st.push_back(n);}
    }
  }
  return out;
}

float cross2(const Eigen::Vector2f & u, const Eigen::Vector2f & v)
{
  return u.x() * v.y() - u.y() * v.x();
}

// Length of segment a-b (XY) inside triangle c (XY), by clipping against its three edges.
float overlap_length(
  const NavMap & nm, NavCelId c, const Eigen::Vector2f & a, const Eigen::Vector2f & b)
{
  Eigen::Vector2f p[3];
  for (int k = 0; k < 3; ++k) {
    const auto v = nm.positions.at(nm.navcels[c].v[k]);
    p[k] = {v.x(), v.y()};
  }
  const float sgn = cross2(p[1] - p[0], p[2] - p[0]) > 0 ? 1.0f : -1.0f;
  float t0 = 0.0f, t1 = 1.0f;
  for (int e = 0; e < 3; ++e) {
    const Eigen::Vector2f edge = p[(e + 1) % 3] - p[e];
    const float s0 = sgn * cross2(edge, a - p[e]);
    const float ds = sgn * cross2(edge, b - a);
    if (std::abs(ds) < 1e-12f) {
      if (s0 < 0) {return 0.0f;}
      continue;
    }
    const float t = -s0 / ds;
    if (ds > 0) {t0 = std::max(t0, t);} else {t1 = std::min(t1, t);}
  }
  return std::max(0.0f, t1 - t0) * (b - a).norm();
}

}  // namespace

// ------------------------------------------------------------------ AABB

TEST(Iterators, AabbMatchesBruteForceOnBothFloors)
{
  const NavMap nm = two_floors();
  std::mt19937 rng(1);
  std::uniform_real_distribution<float> U(-1.0f, 11.0f), Z(-1.0f, 4.0f);
  for (int q = 0; q < 200; ++q) {
    AABB box;
    box.expand({U(rng), U(rng), Z(rng)});
    box.expand({U(rng), U(rng), Z(rng)});
    const auto got = collect([&](auto f) {return navmap::for_each_in_aabb(nm, box, f);});
    std::set<NavCelId> want;
    for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
      if (navmap::detail::boxes_overlap(navmap::detail::navcel_box(nm, c), box)) {want.insert(c);}
    }
    EXPECT_EQ(got, want) << q;
  }
}

TEST(Iterators, AabbSelectsALevelByHeightAndStops)
{
  const NavMap nm = two_floors();
  AABB upper;
  upper.expand({-1, -1, 2.5f});
  upper.expand({11, 11, 3.5f});
  for (NavCelId c : collect([&](auto f) {return navmap::for_each_in_aabb(nm, upper, f);})) {
    EXPECT_NEAR(nm.navcel_centroid(c).z(), 3.0f, 1e-5f);
  }
  EXPECT_EQ(
    navmap::for_each_in_aabb(nm, upper, [](NavCelId) {return true;}), 800u);
  int calls = 0;
  EXPECT_EQ(navmap::for_each_in_aabb(nm, upper, [&](NavCelId) {return ++calls < 7;}), 7u);
  AABB empty;  // inverted (default) box overlaps nothing
  EXPECT_EQ(navmap::for_each_in_aabb(nm, empty, [](NavCelId) {}), 0u);
  AABB away;
  away.expand({50, 50, 0});
  away.expand({60, 60, 1});
  EXPECT_EQ(navmap::for_each_in_aabb(nm, away, [](NavCelId) {}), 0u);
  EXPECT_EQ(navmap::for_each_in_aabb(NavMap{}, upper, [](NavCelId) {}), 0u);
}

// ------------------------------------------------------------------ disc and ring

TEST(Iterators, EuclideanDiscOnAFlatMapIsTheCentroidsWithinTheRadius)
{
  const NavMap nm = flat_map();
  std::mt19937 rng(2);
  std::uniform_real_distribution<float> U(0.2f, 9.8f), R(0.0f, 4.0f);
  for (int q = 0; q < 100; ++q) {
    const Eigen::Vector3f c(U(rng), U(rng), 0.0f);
    const float r = R(rng);
    const NavCelId seed = seed_at(nm, c);
    const auto got = collect(
      [&](auto f) {return navmap::for_each_in_disc(nm, seed, c, r, f);});
    auto want = flood(nm, seed, [&](NavCelId x) {return (nm.navcel_centroid(x) - c).norm() <= r;});
    want.insert(seed);  // always visited: it holds the centre
    EXPECT_EQ(got, want) << q << " r " << r;
  }
}

TEST(Iterators, DiscNeverReachesAnotherFloor)
{
  const NavMap nm = two_floors();
  for (const float z : {0.0f, 3.0f}) {
    const Eigen::Vector3f c(5.1f, 4.9f, z);
    for (const Metric m : {Metric::EUCLIDEAN, Metric::GEODESIC}) {
      const auto got = collect(
        [&](auto f) {return navmap::for_each_in_disc(nm, c, 20.0f, f, m);});
      EXPECT_EQ(got.size(), 800u);  // the whole floor, and only it
      for (NavCelId x : got) {
        EXPECT_NEAR(nm.navcel_centroid(x).z(), z, 1e-5f);
      }
    }
  }
}

TEST(Iterators, DiscEdgeValues)
{
  const NavMap nm = flat_map();
  const Eigen::Vector3f c(5.1f, 4.9f, 0.0f);
  const NavCelId seed = seed_at(nm, c);
  for (const Metric m : {Metric::EUCLIDEAN, Metric::GEODESIC}) {
    // Radius 0, or below the seed's centroid: just the seed
    EXPECT_EQ(
      collect([&](auto f) {return navmap::for_each_in_disc(nm, seed, c, 0.0f, f, m);}),
      std::set<NavCelId>{seed});
    // Negative or NaN radius, bad seed: nothing
    EXPECT_EQ(navmap::for_each_in_disc(nm, seed, c, -1.0f, [](NavCelId) {}, m), 0u);
    EXPECT_EQ(navmap::for_each_in_disc(nm, seed, c, NAN, [](NavCelId) {}, m), 0u);
    EXPECT_EQ(navmap::for_each_in_disc(nm, NavCelId{100000}, c, 1.0f, [](NavCelId) {}, m), 0u);
    // Off the map: nothing
    EXPECT_EQ(
      navmap::for_each_in_disc(nm, Eigen::Vector3f(50, 50, 0), 3.0f, [](NavCelId) {}, m),
      0u);
    // Early stop
    int calls = 0;
    EXPECT_EQ(
      navmap::for_each_in_disc(nm, seed, c, 3.0f, [&](NavCelId) {return ++calls < 5;}, m),
      5u);
  }
  EXPECT_EQ(
    navmap::for_each_in_disc(NavMap{}, Eigen::Vector3f(0, 0, 0), 1.0f, [](NavCelId) {}),
    0u);
}

TEST(Iterators, GeodesicDiscDoesNotCrossAWallThatEuclideanSees)
{
  // A wall (missing column x in [5, 5.5)) with a gap at the far end: the cells right behind the
  // wall are 1 m away in a straight line but ~19 m along the surface.
  NavMap nm;
  add_grid(nm, 20, 20, 0.5f, flat(0.0f), [](int i, int j) {return i == 10 && j < 19;});
  nm.rebuild_geometry_accels();
  const Eigen::Vector3f c(4.75f, 2.0f, 0.0f), behind(5.75f, 2.0f, 0.0f);
  const NavCelId seed = seed_at(nm, c), b = seed_at(nm, behind);
  const auto euclid = collect(
    [&](auto f) {return navmap::for_each_in_disc(nm, seed, c, 3.0f, f, Metric::EUCLIDEAN);});
  const auto geo = collect(
    [&](auto f) {return navmap::for_each_in_disc(nm, seed, c, 3.0f, f, Metric::GEODESIC);});
  // Both grow through the surface, so neither crosses the wall within 3 m...
  EXPECT_FALSE(euclid.count(b));
  EXPECT_FALSE(geo.count(b));
  // ...but with a radius that reaches the gap, Euclidean wraps around and geodesic does not.
  const auto e_far = collect(
    [&](auto f) {return navmap::for_each_in_disc(nm, seed, c, 9.5f, f, Metric::EUCLIDEAN);});
  const auto g_far = collect(
    [&](auto f) {return navmap::for_each_in_disc(nm, seed, c, 9.5f, f, Metric::GEODESIC);});
  EXPECT_TRUE(e_far.count(b));
  EXPECT_FALSE(g_far.count(b));
  for (NavCelId x : g_far) {
    EXPECT_TRUE(e_far.count(x)) << x;
  }                                                             // geodesic >= Euclidean
}

TEST(Iterators, GeodesicDistanceAroundAWallIsTheShortestDetour)
{
  // Wall x in [5, 5.5), y in [0, 9.5), gap above it. From (4.75, 2) to (5.75, 2) the shortest
  // path on the surface goes over the wall's end: about 7.5 + 0.5 + 7.5 = 15.5 m.
  NavMap nm;
  add_grid(nm, 20, 20, 0.5f, flat(0.0f), [](int i, int j) {return i == 10 && j < 19;});
  nm.rebuild_geometry_accels();
  const Eigen::Vector3f c(4.75f, 2.0f, 0.0f);
  const NavCelId seed = seed_at(nm, c), b = seed_at(nm, {5.75f, 2.0f, 0.0f});
  float db = -1.0f;
  navmap::for_each_by_distance(
    nm, seed, c, 30.0f, [&](NavCelId x, float d) {
      if (x == b) {db = d;}
    });
  const Eigen::Vector3f pb = nm.navcel_centroid(b);
  const float detour = (Eigen::Vector3f(5.0f, 9.5f, 0) - c).norm() + 0.5f +
    (pb - Eigen::Vector3f(5.5f, 9.5f, 0)).norm();
  EXPECT_GE(db, detour - 1e-3f);           // never through the wall
  EXPECT_LE(db, detour + 2 * 0.5f);        // bends at centroids: about a NavCel per bend
}

TEST(Iterators, RingIsTheDiscMinusItsInnerPartAndCrossesIt)
{
  const NavMap nm = flat_map();
  const Eigen::Vector3f c(5.1f, 4.9f, 0.0f);
  const NavCelId seed = seed_at(nm, c);
  for (const Metric m : {Metric::EUCLIDEAN, Metric::GEODESIC}) {
    const auto ring = collect(
      [&](auto f) {return navmap::for_each_in_ring(nm, seed, c, 2.0f, 3.0f, f, m);});
    std::set<NavCelId> want;
    navmap::for_each_by_distance(
      nm, seed, c, 3.0f, [&](NavCelId x, float d) {
        if (m == Metric::GEODESIC ? d >= 2.0f : (nm.navcel_centroid(x) - c).norm() >= 2.0f) {
          want.insert(x);
        }
      });
    if (m == Metric::EUCLIDEAN) {
      want = flood(nm, seed, [&](NavCelId x) {return (nm.navcel_centroid(x) - c).norm() <= 3.0f;});
      std::erase_if(want, [&](NavCelId x) {return (nm.navcel_centroid(x) - c).norm() < 2.0f;});
    }
    EXPECT_EQ(ring, want);
    EXPECT_FALSE(ring.count(seed));
    EXPECT_GT(ring.size(), 50u);  // the inner 2 m were crossed to get there
  }
  // Empty ring (r_in > r_out) and r_in = 0 (= disc without forcing the seed)
  EXPECT_EQ(navmap::for_each_in_ring(nm, seed, c, 3.0f, 2.0f, [](NavCelId) {}), 0u);
  EXPECT_EQ(
    navmap::for_each_in_ring(nm, seed, c, 0.0f, 3.0f, [](NavCelId) {}),
    navmap::for_each_in_disc(nm, seed, c, 3.0f, [](NavCelId) {}));
}

// ------------------------------------------------------------------ distance order

TEST(Iterators, ByDistanceIsOrderedCompleteAndNeverShorterThanAStraightLine)
{
  const NavMap nm = flat_map();
  const Eigen::Vector3f c(3.3f, 6.1f, 0.0f);
  const NavCelId seed = seed_at(nm, c);
  std::vector<std::pair<NavCelId, float>> seq;
  const std::size_t n = navmap::for_each_by_distance(
    nm, seed, c, 4.0f, [&](NavCelId x, float d) {
      seq.emplace_back(x, d);
    });
  ASSERT_EQ(n, seq.size());
  ASSERT_FALSE(seq.empty());
  EXPECT_EQ(seq.front().first, seed);
  std::set<NavCelId> ids;
  for (std::size_t k = 0; k < seq.size(); ++k) {
    EXPECT_TRUE(ids.insert(seq[k].first).second);
    if (k > 0) {EXPECT_GE(seq[k].second, seq[k - 1].second - 0.5f);}  // up to a neighbour step
    EXPECT_GE(seq[k].second, (nm.navcel_centroid(seq[k].first) - c).norm() - 1e-4f);
    if (k > 0) {EXPECT_LE(seq[k].second, 4.0f);}
  }
  // On a flat open grid the geodesic is the straight line
  float worst = 0.0f;
  for (const auto & [x, d] : seq) {
    const float straight = (nm.navcel_centroid(x) - c).norm();
    if (straight > 1.0f) {worst = std::max(worst, d / straight);}
  }
  std::printf("worst geodesic / straight on a flat grid: %.3f\n", worst);
  EXPECT_LE(worst, 1.001f);
  // Every NavCel with a short straight line to the centre is there
  for (NavCelId x = 0; x < nm.navcels.size(); ++x) {
    if ((nm.navcel_centroid(x) - c).norm() <= 3.0f) {EXPECT_TRUE(ids.count(x)) << x;}
  }
  int calls = 0;
  EXPECT_EQ(
    navmap::for_each_by_distance(
      nm, seed, c, 4.0f, [&](NavCelId, float) {
        return ++calls < 3;
      }), 3u);
}

TEST(Iterators, ByDistanceOnASlopeMeasuresAlongTheSurface)
{
  // 30 degree slope along x: a cell 4 m away in XY is 4 / cos(30) = 4.62 m away on the surface.
  NavMap nm;
  const float k = std::tan(30.0f * static_cast<float>(M_PI) / 180.0f);
  add_grid(nm, 40, 4, 0.25f, [k](float x, float) {return k * x;});
  nm.rebuild_geometry_accels();
  const Eigen::Vector3f c(1.0f, 0.5f, k * 1.0f), far(5.0f, 0.5f, k * 5.0f);
  const NavCelId seed = seed_at(nm, c), target = seed_at(nm, far);
  float d_target = -1.0f;
  navmap::for_each_by_distance(
    nm, seed, c, 10.0f, [&](NavCelId x, float d) {
      if (x == target) {d_target = d;}
    });
  const float straight = (nm.navcel_centroid(target) - c).norm();
  EXPECT_NEAR(d_target, straight, 1e-3f);  // a plane: the straight line is the geodesic
  EXPECT_GT(d_target, 4.4f);  // the XY distance (4 m) would be wrong
}

// ------------------------------------------------------------------ polygon

TEST(Iterators, PolygonMatchesBruteForceIncludingConcaveOnes)
{
  const NavMap nm = flat_map();
  const std::vector<std::vector<Eigen::Vector2f>> polys = {
    {{1, 1}, {6, 1}, {6, 4}, {1, 4}},                       // rectangle
    {{1, 1}, {8, 1}, {8, 3}, {3, 3}, {3, 8}, {1, 8}},       // L (concave)
    {{5, 0.5f}, {9.5f, 5}, {5, 9.5f}, {0.5f, 5}},           // rotated square
  };
  for (const auto & poly : polys) {
    const Eigen::Vector3f inside(poly[0].x() + 0.3f, poly[0].y() + 0.4f, 0.0f);
    const NavCelId seed = seed_at(nm, inside);
    const auto got = collect(
      [&](auto f) {return navmap::for_each_in_polygon(nm, seed, poly, f);});
    std::set<NavCelId> want;
    for (NavCelId x = 0; x < nm.navcels.size(); ++x) {
      const auto p = nm.navcel_centroid(x);
      if (navmap::detail::in_polygon(poly, p.x(), p.y())) {want.insert(x);}
    }
    EXPECT_EQ(got, want);
  }
}

TEST(Iterators, PolygonStaysOnTheSeedLevelAndHandlesDegenerateInput)
{
  const NavMap nm = two_floors();
  const std::vector<Eigen::Vector2f> sq{{2, 2}, {6, 2}, {6, 6}, {2, 6}};
  for (const float z : {0.0f, 3.0f}) {
    const auto got = collect(
      [&](auto f) {
        return navmap::for_each_in_polygon(nm, Eigen::Vector3f(4, 4, z), sq, f);
      });
    EXPECT_EQ(got.size(), 2u * 8u * 8u);
    for (NavCelId x : got) {
      EXPECT_NEAR(nm.navcel_centroid(x).z(), z, 1e-5f);
    }
  }
  const NavCelId seed = seed_at(nm, {4, 4, 0});
  EXPECT_EQ(navmap::for_each_in_polygon(nm, seed, {{0, 0}, {1, 1}}, [](NavCelId) {}), 0u);
  EXPECT_EQ(navmap::for_each_in_polygon(nm, seed, {}, [](NavCelId) {}), 0u);
  // A polygon away from the seed is not reached (regions grow from the seed)
  EXPECT_EQ(
    navmap::for_each_in_polygon(
      nm, seed, {{8, 8}, {9, 8}, {9, 9}, {8, 9}}, [](NavCelId) {}), 0u);
  EXPECT_EQ(
    navmap::for_each_in_polygon(
      nm, Eigen::Vector3f(50, 50, 0), sq, [](NavCelId) {}), 0u);
  int calls = 0;
  EXPECT_EQ(navmap::for_each_in_polygon(nm, seed, sq, [&](NavCelId) {return ++calls < 4;}), 4u);
}

// ------------------------------------------------------------------ segment walk

namespace
{

struct Walk
{
  bool reached;
  std::vector<std::tuple<NavCelId, float, float>> cells;
};

Walk walk(const NavMap & nm, const Eigen::Vector3f & a, const Eigen::Vector3f & b)
{
  Walk w;
  w.reached = navmap::for_each_on_segment(
    nm, a, b, [&](NavCelId c, float t0, float t1) {
      w.cells.emplace_back(c, t0, t1);
    });
  return w;
}

}  // namespace

TEST(Iterators, SegmentWalkCoversTheSegmentAndMatchesTheCrossedCells)
{
  const NavMap nm = flat_map();
  std::mt19937 rng(3);
  std::uniform_real_distribution<float> U(0.05f, 9.95f);
  for (int q = 0; q < 300; ++q) {
    const Eigen::Vector3f a(U(rng), U(rng), 0.0f), b(U(rng), U(rng), 0.0f);
    const Walk w = walk(nm, a, b);
    ASSERT_TRUE(w.reached) << q;
    ASSERT_FALSE(w.cells.empty());
    EXPECT_FLOAT_EQ(std::get<1>(w.cells.front()), 0.0f);
    EXPECT_FLOAT_EQ(std::get<2>(w.cells.back()), 1.0f);
    std::set<NavCelId> seen;
    for (std::size_t k = 0; k < w.cells.size(); ++k) {
      const auto [c, t0, t1] = w.cells[k];
      EXPECT_TRUE(seen.insert(c).second) << "cell visited twice";
      EXPECT_LE(t0, t1);
      if (k > 0) {EXPECT_NEAR(t0, std::get<2>(w.cells[k - 1]), 1e-5f);}  // contiguous
    }
    // Same set as every triangle the segment crosses with positive length
    std::set<NavCelId> want;
    for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
      if (overlap_length(nm, c, a.head<2>(), b.head<2>()) > 1e-4f) {want.insert(c);}
    }
    for (NavCelId c : want) {
      EXPECT_TRUE(seen.count(c)) << q << " missing " << c;
    }
    // Every visited cell holds the middle of its own interval (crossed, or touched at a vertex)
    for (const auto & [c, t0, t1] : w.cells) {
      const Eigen::Vector3f m = a + 0.5f * (t0 + t1) * (b - a);
      const Eigen::Vector2f e(1e-3f, 0.0f);
      EXPECT_TRUE(
        want.count(c) ||
        overlap_length(nm, c, m.head<2>() - e, m.head<2>() + e) > 0.0f ||
        overlap_length(nm, c, m.head<2>() - e.reverse(), m.head<2>() + e.reverse()) > 0.0f)
        << q << " cell " << c << " not on the segment";
    }
  }
}

TEST(Iterators, SegmentWalkAlongMeshEdgesAndThroughVertices)
{
  const NavMap nm = flat_map();
  // Along the diagonals of the grid (exactly on triangle edges) and along grid lines
  for (const auto & [a, b] : std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>{
    {{0.5f, 0.5f, 0}, {9.5f, 9.5f, 0}}, {{9.5f, 9.5f, 0}, {0.5f, 0.5f, 0}},
    {{1.0f, 0.25f, 0}, {1.0f, 9.75f, 0}}, {{0.25f, 3.0f, 0}, {9.75f, 3.0f, 0}},
    {{2.0f, 2.0f, 0}, {2.0f, 2.0f, 0}}})
  {
    const Walk w = walk(nm, a, b);
    EXPECT_TRUE(w.reached) << a.transpose() << " -> " << b.transpose();
    EXPECT_LE(w.cells.size(), 4u * 20u);  // no loops
    EXPECT_FLOAT_EQ(std::get<2>(w.cells.back()), 1.0f);
  }
  const Walk point = walk(nm, {2.2f, 2.3f, 0}, {2.2f, 2.3f, 0});  // a == b: one cell
  ASSERT_EQ(point.cells.size(), 1u);
  EXPECT_FLOAT_EQ(std::get<1>(point.cells[0]), 0.0f);
  EXPECT_FLOAT_EQ(std::get<2>(point.cells[0]), 1.0f);
}

TEST(Iterators, SegmentWalkStopsAtBordersAndHoles)
{
  NavMap nm;
  add_grid(nm, 20, 20, 0.5f, flat(0.0f), [](int i, int j) {return i == 10 && j >= 5 && j < 15;});
  nm.rebuild_geometry_accels();
  // Across the hole
  Walk w = walk(nm, {2, 5, 0}, {8, 5, 0});
  EXPECT_FALSE(w.reached);
  ASSERT_FALSE(w.cells.empty());
  EXPECT_LT(std::get<2>(w.cells.back()), 0.55f);
  EXPECT_GT(std::get<2>(w.cells.back()), 0.45f);
  // Leaving the map
  w = walk(nm, {2, 1, 0}, {15, 1, 0});
  EXPECT_FALSE(w.reached);
  EXPECT_NEAR(std::get<2>(w.cells.back()) * 13.0f + 2.0f, 10.0f, 1e-3f);
  // Around the hole, below it: reaches
  EXPECT_TRUE(walk(nm, {2, 1, 0}, {8, 1, 0}).reached);
  // Off the map start
  EXPECT_FALSE(walk(nm, {-3, 1, 0}, {2, 1, 0}).reached);
  // Early stop
  int calls = 0;
  EXPECT_FALSE(
    navmap::for_each_on_segment(
      nm, Eigen::Vector3f(1, 1, 0), Eigen::Vector3f(9, 1, 0),
      [&](NavCelId, float, float) {return ++calls < 3;}));
  EXPECT_EQ(calls, 3);
}

TEST(Iterators, SegmentWalkStaysOnItsFloorAndFollowsSlopes)
{
  const NavMap nm = two_floors();
  for (const float z : {0.0f, 3.0f}) {
    const Walk w = walk(nm, {1, 1, z}, {9, 8, z});
    EXPECT_TRUE(w.reached);
    for (const auto & [c, t0, t1] : w.cells) {
      EXPECT_NEAR(nm.navcel_centroid(c).z(), z, 1e-5f);
    }
  }
  NavMap ramp;
  add_grid(ramp, 40, 4, 0.25f, [](float x, float) {return 0.3f * x;});
  ramp.rebuild_geometry_accels();
  const Walk up = walk(ramp, {0.5f, 0.5f, 0.15f}, {9.5f, 0.6f, 2.85f});
  EXPECT_TRUE(up.reached);
  float zprev = -1.0f;
  for (const auto & [c, t0, t1] : up.cells) {
    const float zc = ramp.navcel_centroid(c).z();
    EXPECT_GE(zc, zprev - 0.1f);  // climbs
    zprev = zc;
  }
}

// ------------------------------------------------------------------ scratch reuse

TEST(Iterators, ScratchReuseGivesTheSameResultsAcrossMapsAndCalls)
{
  const NavMap big = flat_map(30), small = flat_map(6);
  TraversalScratch sc;
  for (int round = 0; round < 3; ++round) {
    for (const NavMap * nm : {&big, &small}) {
      const Eigen::Vector3f c(1.3f, 1.6f, 0.0f);
      const NavCelId seed = seed_at(*nm, c);
      for (const Metric m : {Metric::EUCLIDEAN, Metric::GEODESIC}) {
        const auto with = collect(
          [&](auto f) {return navmap::for_each_in_disc(*nm, seed, c, 1.5f, f, m, &sc);});
        const auto without = collect(
          [&](auto f) {return navmap::for_each_in_disc(*nm, seed, c, 1.5f, f, m);});
        EXPECT_EQ(with, without);
      }
      const std::vector<Eigen::Vector2f> sq{{0.6f, 0.6f}, {2, 0.6f}, {2, 2}, {0.6f, 2}};
      EXPECT_EQ(
        navmap::for_each_in_polygon(*nm, seed, sq, [](NavCelId) {}, &sc),
        navmap::for_each_in_polygon(*nm, seed, sq, [](NavCelId) {}));
      AABB box;
      box.expand({0, 0, -1});
      box.expand({2, 2, 1});
      EXPECT_EQ(
        navmap::for_each_in_aabb(*nm, box, [](NavCelId) {}, &sc),
        navmap::for_each_in_aabb(*nm, box, [](NavCelId) {}));
    }
  }
}
