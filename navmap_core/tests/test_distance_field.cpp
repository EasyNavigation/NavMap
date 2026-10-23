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
#include <vector>

#include "navmap_core/NavMap.hpp"

using navmap::NavCelId;
using navmap::NavMap;

namespace
{

constexpr float kInf = std::numeric_limits<float>::infinity();

// nx x ny cells of `res` m at z = f(x, y), skipping cells where skip(i, j); returns the surface.
std::size_t add_grid(
  NavMap & nm, int nx, int ny, float res, const std::function<float(float, float)> & f,
  const std::function<bool(int, int)> & skip = nullptr)
{
  const std::size_t s = nm.create_surface("map");
  const auto base = static_cast<uint32_t>(nm.positions.size());
  for (int j = 0; j <= ny; ++j) {
    for (int i = 0; i <= nx; ++i) {
      nm.add_vertex({i * res, j * res, f(i * res, j * res)});
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

float point_segment(const Eigen::Vector3f & p, const Eigen::Vector3f & a, const Eigen::Vector3f & b)
{
  const Eigen::Vector3f ab = b - a;
  const float t = std::clamp((p - a).dot(ab) / ab.squaredNorm(), 0.0f, 1.0f);
  return (p - (a + t * ab)).norm();
}

// Exact nearest distance over all seeds (no connectivity).
std::vector<float> brute(const NavMap & nm, const std::vector<NavCelId> & seeds, bool boundary)
{
  std::vector<float> d(nm.navcels.size(), kInf);
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    const auto p = nm.navcel_centroid(c);
    for (NavCelId s : seeds) {
      d[c] = std::min(d[c], (p - nm.navcel_centroid(s)).norm());
    }
    if (!boundary) {continue;}
    for (NavCelId b = 0; b < nm.navcels.size(); ++b) {
      for (int e = 0; e < 3; ++e) {
        if (nm.navcels[b].neighbor[e] != std::numeric_limits<NavCelId>::max()) {continue;}
        d[c] = std::min(
          d[c], point_segment(
            p, nm.positions.at(nm.navcels[b].v[e]), nm.positions.at(nm.navcels[b].v[(e + 1) % 3])));
      }
    }
  }
  return d;
}

NavCelId cell_near(const NavMap & nm, float x, float y, float z)
{
  std::size_t s; NavCelId c; Eigen::Vector3f bary, hit;
  EXPECT_TRUE(nm.locate_navcel({x, y, z}, s, c, bary, &hit));
  return c;
}

auto flat(float h) {return [h](float, float) {return h;};}

}  // namespace

TEST(NavMap_DistanceField, BoundaryOfARectangleIsExact)
{
  NavMap nm;
  add_grid(nm, 20, 12, 0.5f, flat(0.0f));
  nm.rebuild_geometry_accels();
  const auto d = nm.distance_field({}, true);
  const auto ref = brute(nm, {}, true);
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    const auto p = nm.navcel_centroid(c);
    const float expected = std::min({p.x(), 10.0f - p.x(), p.y(), 6.0f - p.y()});
    EXPECT_NEAR(d[c], expected, 1e-4f) << c;
    EXPECT_NEAR(d[c], ref[c], 1e-4f) << c;
  }
}

TEST(NavMap_DistanceField, SeedCellsGiveStraightLineDistance)
{
  NavMap nm;
  add_grid(nm, 20, 20, 0.5f, flat(0.0f));
  nm.rebuild_geometry_accels();
  const std::vector<NavCelId> seeds{cell_near(nm, 2.1f, 2.2f, 0.0f),
    cell_near(nm, 7.3f, 6.9f, 0.0f)};
  const auto d = nm.distance_field(seeds, false);
  const auto ref = brute(nm, seeds, false);
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    EXPECT_NEAR(d[c], ref[c], 1e-4f) << c;
  }
  for (NavCelId s : seeds) {
    EXPECT_FLOAT_EQ(d[s], 0.0f);
  }

  // Seeds and boundary together: the nearest of both.
  const auto both = nm.distance_field(seeds, true);
  const auto ref_both = brute(nm, seeds, true);
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    EXPECT_NEAR(both[c], ref_both[c], 1e-4f) << c;
  }
}

TEST(NavMap_DistanceField, MaxDistanceCutsOff)
{
  NavMap nm;
  add_grid(nm, 20, 20, 0.5f, flat(0.0f));
  nm.rebuild_geometry_accels();
  const NavCelId seed = cell_near(nm, 5.1f, 5.2f, 0.0f);
  const auto full = nm.distance_field({seed}, false);
  const auto cut = nm.distance_field({seed}, false, 2.0f);
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    if (full[c] <= 2.0f) {
      EXPECT_NEAR(cut[c], full[c], 1e-5f);
    } else {
      EXPECT_EQ(cut[c], kInf);
    }
  }
}

TEST(NavMap_DistanceField, StackedFloorsDoNotSeeEachOther)
{
  NavMap nm;
  add_grid(nm, 10, 10, 0.5f, flat(0.0f));
  add_grid(nm, 10, 10, 0.5f, flat(2.5f));
  nm.rebuild_geometry_accels();
  const NavCelId low = cell_near(nm, 2.6f, 2.6f, 0.0f);
  const auto d = nm.distance_field({low}, false);
  for (auto c : nm.surfaces[1].navcels) {
    EXPECT_EQ(d[c], kInf);
  }
  for (auto c : nm.surfaces[0].navcels) {
    EXPECT_LT(d[c], kInf);
  }

  // Boundary of the upper floor only reaches the upper floor's own cells.
  const NavCelId high = cell_near(nm, 2.6f, 2.6f, 2.5f);
  const auto b = nm.distance_field({}, true);
  EXPECT_NEAR(b[high], b[low], 1e-4f);  // same geometry, same distance to its own border
}

TEST(NavMap_DistanceField, SlopeDistanceIs3D)
{
  // 30 deg ramp along x: the distance to a seed grows with the slope length, not the XY one.
  const float k = std::tan(30.0f * static_cast<float>(M_PI) / 180.0f);
  NavMap nm;
  add_grid(nm, 20, 4, 0.5f, [k](float x, float) {return k * x;});
  nm.rebuild_geometry_accels();
  const NavCelId seed = cell_near(nm, 0.4f, 1.0f, 0.2f);
  const auto d = nm.distance_field({seed}, false);
  const auto ref = brute(nm, {seed}, false);
  const NavCelId far = cell_near(nm, 8.4f, 1.0f, k * 8.4f);
  EXPECT_NEAR(d[far], ref[far], 1e-4f);
  const auto pf = nm.navcel_centroid(far), ps = nm.navcel_centroid(seed);
  EXPECT_GT(d[far], (pf.head<2>() - ps.head<2>()).norm() * 1.1f);
}

TEST(NavMap_DistanceField, InteriorHoleEdgesAreBoundary)
{
  NavMap nm;
  auto hole = [](int i, int j) {return i >= 9 && i <= 10 && j >= 9 && j <= 10;};
  add_grid(nm, 20, 20, 0.5f, flat(0.0f), hole);
  nm.rebuild_geometry_accels();
  const auto d = nm.distance_field({}, true);
  const auto ref = brute(nm, {}, true);
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    EXPECT_NEAR(d[c], ref[c], 1e-4f) << c;
  }
  // Next to the 1x1 m hole (x, y in [4.5, 5.5]).
  EXPECT_LT(d[cell_near(nm, 4.2f, 5.0f, 0.0f)], 0.31f);
}

TEST(NavMap_DistanceField, NeverBelowTheTrueDistanceOnIrregularTerrain)
{
  std::mt19937 rng(17);
  std::uniform_real_distribution<float> U(0.0f, 1.0f);
  for (int m = 0; m < 4; ++m) {
    const float a = 0.3f * U(rng), b = 0.3f * U(rng);
    NavMap nm;
    add_grid(
      nm, 30, 25, 0.4f, [&](float x, float y) {return a * std::sin(x) + b * std::cos(0.7f * y);},
      [&](int, int) {return U(rng) < 0.05f;});  // scattered holes
    nm.rebuild_geometry_accels();
    std::vector<NavCelId> seeds;
    for (int k = 0; k < 5; ++k) {
      seeds.push_back(static_cast<NavCelId>(U(rng) * static_cast<float>(nm.navcels.size() - 1)));
    }
    const auto d = nm.distance_field(seeds, true);
    const auto ref = brute(nm, seeds, true);
    double sum = 0.0;
    int n = 0;
    for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
      if (!std::isfinite(d[c])) {continue;}
      EXPECT_GE(d[c], ref[c] - 1e-4f) << "map " << m << " cell " << c;
      sum += d[c] - ref[c];
      ++n;
    }
    EXPECT_LT(sum / n, 0.02) << "map " << m;  // propagation is close to the exact distance
  }
}

TEST(NavMap_DistanceField, DegenerateInputs)
{
  NavMap empty;
  empty.rebuild_geometry_accels();
  EXPECT_TRUE(empty.distance_field({0, 5}, true).empty());

  NavMap nm;
  add_grid(nm, 4, 4, 0.5f, flat(0.0f));
  nm.rebuild_geometry_accels();
  for (float v : nm.distance_field({}, false)) {
    EXPECT_EQ(v, kInf);
  }
  const auto d = nm.distance_field({9999}, false);  // out of range: ignored
  for (float v : d) {
    EXPECT_EQ(v, kInf);
  }
  const auto z = nm.distance_field({0}, false, 0.0f);  // only the seed itself
  EXPECT_FLOAT_EQ(z[0], 0.0f);
  EXPECT_EQ(std::count_if(z.begin(), z.end(), [](float v) {return v < kInf;}), 1);
}
