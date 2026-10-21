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

// Randomized checks of the geometric queries against brute-force references.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <vector>

#include "navmap_core/NavMap.hpp"

using navmap::NavCelId;
using navmap::NavMap;

namespace
{

constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
// Reference rays start low enough to keep float precision (all maps are below 20 m).
constexpr float kTopZ = 50.0f;

struct Patch
{
  float x0, y0, res, base_z, sx, sy, bump;
  int nx, ny;
  float z(float x, float y) const
  {
    return base_z + sx * x + sy * y + bump * std::sin(1.3f * x) * std::cos(0.7f * y);
  }
};

// Random multi-surface map: sloped, bumpy patches overlapping in XY, with the
// triangles of all surfaces interleaved in the global NavCel array.
NavMap random_map(std::mt19937 & rng, int n_surfaces)
{
  std::uniform_real_distribution<float> U(0.0f, 1.0f);
  NavMap nm;
  struct Tri {uint32_t a, b, c; std::size_t s;};
  std::vector<Tri> tris;

  for (int k = 0; k < n_surfaces; ++k) {
    Patch p;
    p.res = 0.2f + 0.8f * U(rng);
    p.nx = 4 + static_cast<int>(16 * U(rng));
    p.ny = 4 + static_cast<int>(16 * U(rng));
    p.x0 = -2.0f + 4.0f * U(rng);
    p.y0 = -2.0f + 4.0f * U(rng);
    p.base_z = 2.5f * static_cast<float>(k) + 0.3f * U(rng);
    p.sx = -0.2f + 0.4f * U(rng);
    p.sy = -0.2f + 0.4f * U(rng);
    p.bump = 0.15f * U(rng);

    const std::size_t s = nm.create_surface("map");
    const auto base = static_cast<uint32_t>(nm.positions.size());
    for (int j = 0; j <= p.ny; ++j) {
      for (int i = 0; i <= p.nx; ++i) {
        const float x = p.x0 + i * p.res, y = p.y0 + j * p.res;
        nm.add_vertex({x, y, p.z(x, y)});
      }
    }
    auto v = [&](int i, int j) {return base + static_cast<uint32_t>(j * (p.nx + 1) + i);};
    for (int j = 0; j < p.ny; ++j) {
      for (int i = 0; i < p.nx; ++i) {
        tris.push_back({v(i, j), v(i + 1, j), v(i + 1, j + 1), s});
        tris.push_back({v(i, j), v(i + 1, j + 1), v(i, j + 1), s});
      }
    }
  }

  std::shuffle(tris.begin(), tris.end(), rng);
  for (const auto & t : tris) {
    nm.add_navcel_to_surface(t.s, nm.add_navcel(t.a, t.b, t.c));
  }
  nm.rebuild_geometry_accels();
  return nm;
}

Eigen::Vector3f vtx(const NavMap & nm, NavCelId cid, int k)
{
  return nm.positions.at(nm.navcels[cid].v[k]);
}

// Heights of the surface under (x, y), using the same ray test as the library.
std::vector<float> heights_at(const NavMap & nm, float x, float y)
{
  std::vector<float> out;
  const Eigen::Vector3f o(x, y, kTopZ), d(0.0f, 0.0f, -1.0f);
  for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
    float t, u, v;
    if (navmap::ray_triangle_intersect(o, d, vtx(nm, cid, 0), vtx(nm, cid, 1), vtx(nm, cid, 2),
      t, u, v))
    {
      out.push_back(o.z() - t);
    }
  }
  return out;
}

std::optional<float> brute_raycast(
  const NavMap & nm, const Eigen::Vector3f & o,
  const Eigen::Vector3f & d)
{
  std::optional<float> best;
  for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
    float t, u, v;
    if (navmap::ray_triangle_intersect(o, d, vtx(nm, cid, 0), vtx(nm, cid, 1), vtx(nm, cid, 2),
      t, u, v) && (!best || t < *best))
    {
      best = t;
    }
  }
  return best;
}

float brute_closest_sq(const NavMap & nm, const Eigen::Vector3f & p)
{
  float best = kInf;
  for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
    best = std::min(
      best, navmap::point_triangle_squared_distance(
        p, vtx(nm, cid, 0), vtx(nm, cid, 1), vtx(nm, cid, 2)));
  }
  return best;
}

// True if (x, y) projects inside triangle cid (vertical ray hits it).
bool covers_xy(const NavMap & nm, NavCelId cid, float x, float y)
{
  float t, u, v;
  return navmap::ray_triangle_intersect(
    {x, y, kTopZ}, {0.0f, 0.0f, -1.0f},
    vtx(nm, cid, 0), vtx(nm, cid, 1), vtx(nm, cid, 2), t, u, v);
}

struct Bounds
{
  Eigen::Vector3f min{kInf, kInf, kInf}, max{-kInf, -kInf, -kInf};
};

Bounds bounds_of(const NavMap & nm)
{
  Bounds b;
  for (std::size_t i = 0; i < nm.positions.size(); ++i) {
    b.min = b.min.cwiseMin(nm.positions.at(static_cast<uint32_t>(i)));
    b.max = b.max.cwiseMax(nm.positions.at(static_cast<uint32_t>(i)));
  }
  return b;
}

}  // namespace

TEST(NavMap_Properties, LocateMatchesBruteForceNearestSurface)
{
  std::mt19937 rng(1234);
  int checked = 0, on_surface_checked = 0;
  for (int map_i = 0; map_i < 12; ++map_i) {
    const NavMap nm = random_map(rng, 1 + map_i % 4);
    const Bounds b = bounds_of(nm);
    std::uniform_real_distribution<float> X(b.min.x() - 1.0f, b.max.x() + 1.0f);
    std::uniform_real_distribution<float> Y(b.min.y() - 1.0f, b.max.y() + 1.0f);
    std::uniform_real_distribution<float> Z(b.min.z() - 1.0f, b.max.z() + 1.0f);
    std::bernoulli_distribution on_surface(0.3);

    for (int q = 0; q < 150; ++q) {
      const float x = X(rng), y = Y(rng);
      const auto hs = heights_at(nm, x, y);
      float z = Z(rng);
      const bool exact = !hs.empty() && on_surface(rng);
      if (exact) {
        z = hs[std::uniform_int_distribution<std::size_t>(0, hs.size() - 1)(rng)];
      }

      std::size_t s; NavCelId cid; Eigen::Vector3f bary, hit;
      const bool ok = nm.locate_navcel({x, y, z}, s, cid, bary, &hit);
      if (hs.empty()) {
        EXPECT_FALSE(ok) << "no surface under (" << x << "," << y << ")";
        continue;
      }

      // Nearest surface in |dz|; skip ambiguous ties between distinct heights.
      std::vector<float> dz;
      for (float h : hs) {
        dz.push_back(std::fabs(h - z));
      }
      std::sort(dz.begin(), dz.end());
      // Same height twice is a shared edge; a near-equal different one is a tie.
      const auto other = std::find_if(
        dz.begin(), dz.end(), [&](float v) {return v - dz[0] > 1e-5f;});
      if (other != dz.end() && *other - dz[0] < 1e-3f) {continue;}

      ASSERT_TRUE(ok) << "(" << x << "," << y << "," << z << ")";
      EXPECT_NEAR(std::fabs(hit.z() - z), dz[0], 2e-4f) << "(" << x << "," << y << "," << z << ")";
      EXPECT_EQ(nm.surface_of(cid), s);
      EXPECT_TRUE(covers_xy(nm, cid, x, y));
      ++checked;
      on_surface_checked += exact;
    }
  }
  EXPECT_GT(checked, 500);
  EXPECT_GT(on_surface_checked, 100);
}

TEST(NavMap_Properties, LocateWithHintIsConsistent)
{
  std::mt19937 rng(99);
  for (int map_i = 0; map_i < 6; ++map_i) {
    const NavMap nm = random_map(rng, 2 + map_i % 3);
    const Bounds b = bounds_of(nm);
    std::uniform_real_distribution<float> X(b.min.x(), b.max.x());
    std::uniform_real_distribution<float> Y(b.min.y(), b.max.y());
    std::uniform_int_distribution<NavCelId> C(0, static_cast<NavCelId>(nm.navcels.size() - 1));

    for (int q = 0; q < 200; ++q) {
      const float x = X(rng), y = Y(rng);
      const auto hs = heights_at(nm, x, y);
      if (hs.empty()) {continue;}
      const float z = hs[0] + 0.05f;

      std::size_t s0; NavCelId c0; Eigen::Vector3f bary, hit0;
      ASSERT_TRUE(nm.locate_navcel({x, y, z}, s0, c0, bary, &hit0));

      // Hint on the right cell: same answer.
      NavMap::LocateOpts opts;
      opts.hint_cid = c0;
      std::size_t s1; NavCelId c1; Eigen::Vector3f hit1;
      ASSERT_TRUE(nm.locate_navcel({x, y, z}, s1, c1, bary, &hit1, opts));
      EXPECT_EQ(s1, s0);
      EXPECT_NEAR(hit1.z(), hit0.z(), 2e-4f);

      // Random hint: either the nearest surface or a cell within height_eps of the point.
      opts.hint_cid = C(rng);
      std::size_t s2; NavCelId c2; Eigen::Vector3f hit2;
      ASSERT_TRUE(nm.locate_navcel({x, y, z}, s2, c2, bary, &hit2, opts));
      EXPECT_EQ(nm.surface_of(c2), s2);
      const auto & n = nm.navcels[c2].normal;
      const float plane_dist = std::fabs(n.dot(Eigen::Vector3f(x, y, z) - vtx(nm, c2, 0)));
      EXPECT_TRUE(
        std::fabs(hit2.z() - hit0.z()) < 2e-4f || plane_dist <= opts.height_eps + 1e-4f)
        << "hint " << *opts.hint_cid << " plane_dist " << plane_dist;
    }
  }
}

TEST(NavMap_Properties, RaycastMatchesBruteForce)
{
  std::mt19937 rng(7);
  std::normal_distribution<float> N(0.0f, 1.0f);
  int hits = 0, misses = 0;
  for (int map_i = 0; map_i < 8; ++map_i) {
    const NavMap nm = random_map(rng, 1 + map_i % 4);
    const Bounds b = bounds_of(nm);
    std::uniform_real_distribution<float> X(b.min.x() - 2.0f, b.max.x() + 2.0f);
    std::uniform_real_distribution<float> Y(b.min.y() - 2.0f, b.max.y() + 2.0f);
    std::uniform_real_distribution<float> Z(b.min.z() - 2.0f, b.max.z() + 2.0f);

    std::vector<navmap::Ray> rays;
    for (int q = 0; q < 150; ++q) {
      Eigen::Vector3f d(N(rng), N(rng), N(rng));
      if (d.norm() < 1e-3f) {continue;}
      rays.push_back({{X(rng), Y(rng), Z(rng)}, d.normalized()});
    }

    std::vector<navmap::RayHit> many_closest, many_first;
    nm.raycast_many(rays, many_closest, false);
    nm.raycast_many(rays, many_first, true);
    ASSERT_EQ(many_closest.size(), rays.size());

    for (std::size_t i = 0; i < rays.size(); ++i) {
      const auto ref = brute_raycast(nm, rays[i].o, rays[i].d);
      NavCelId cid; float t; Eigen::Vector3f p;
      const bool hit = nm.raycast(rays[i].o, rays[i].d, cid, t, p);
      ASSERT_EQ(hit, ref.has_value()) << "ray " << i;
      EXPECT_EQ(many_closest[i].hit, hit);
      EXPECT_EQ(many_first[i].hit, hit);
      if (!hit) {++misses; continue;}
      ++hits;
      EXPECT_NEAR(t, *ref, 1e-4f);
      EXPECT_LT((p - (rays[i].o + t * rays[i].d)).norm(), 1e-4f);
      EXPECT_NEAR(many_closest[i].t, *ref, 1e-4f);
      EXPECT_EQ(nm.surface_of(many_closest[i].cid), many_closest[i].surface);
      EXPECT_GE(many_first[i].t, *ref - 1e-4f);
    }
  }
  EXPECT_GT(hits, 100);
  EXPECT_GT(misses, 100);
}

TEST(NavMap_Properties, ClosestMatchesBruteForce)
{
  std::mt19937 rng(42);
  for (int map_i = 0; map_i < 8; ++map_i) {
    const NavMap nm = random_map(rng, 1 + map_i % 4);
    const Bounds b = bounds_of(nm);
    std::uniform_real_distribution<float> X(b.min.x() - 3.0f, b.max.x() + 3.0f);
    std::uniform_real_distribution<float> Y(b.min.y() - 3.0f, b.max.y() + 3.0f);
    std::uniform_real_distribution<float> Z(b.min.z() - 3.0f, b.max.z() + 3.0f);
    for (int q = 0; q < 100; ++q) {
      const Eigen::Vector3f p(X(rng), Y(rng), Z(rng));
      std::size_t s; NavCelId cid; Eigen::Vector3f cp; float sq;
      ASSERT_TRUE(nm.closest_navcel(p, s, cid, cp, sq));
      EXPECT_NEAR(sq, brute_closest_sq(nm, p), 1e-4f * std::max(1.0f, sq));
      EXPECT_NEAR((cp - p).squaredNorm(), sq, 1e-3f * std::max(1.0f, sq));
      EXPECT_EQ(nm.surface_of(cid), s);

      // Restricted search never beats the unrestricted one.
      std::size_t s_r; NavCelId c_r; Eigen::Vector3f cp_r; float sq_r;
      ASSERT_TRUE(nm.closest_navcel(p, s_r, c_r, cp_r, sq_r, static_cast<int>(s)));
      EXPECT_EQ(s_r, s);
      EXPECT_NEAR(sq_r, sq, 1e-4f * std::max(1.0f, sq));
    }
  }
}

TEST(NavMap_Properties, AdjacencyIsSymmetricAndSharesAnEdge)
{
  std::mt19937 rng(5);
  for (int map_i = 0; map_i < 6; ++map_i) {
    const NavMap nm = random_map(rng, 1 + map_i % 4);
    for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
      for (NavCelId n : nm.navcel_neighbors(cid)) {
        ASSERT_LT(n, nm.navcels.size());
        const auto back = nm.navcel_neighbors(n);
        EXPECT_NE(std::find(back.begin(), back.end(), cid), back.end());
        int shared = 0;
        for (auto a : nm.navcels[cid].v) {
          for (auto c : nm.navcels[n].v) {
            shared += a == c;
          }
        }
        EXPECT_EQ(shared, 2);
        EXPECT_EQ(nm.surface_of(n), nm.surface_of(cid));
      }
    }
  }
}

TEST(NavMap_Robustness, NonFiniteQueriesFailCleanly)
{
  std::mt19937 rng(3);
  const NavMap nm = random_map(rng, 2);
  const Eigen::Vector3f bad[] = {
    {kNaN, 0.0f, 0.0f}, {0.0f, kNaN, 0.0f}, {0.0f, 0.0f, kNaN},
    {kInf, 0.0f, 0.0f}, {0.0f, -kInf, 0.0f}, {0.0f, 0.0f, kInf}};
  for (const auto & p : bad) {
    std::size_t s; NavCelId cid; Eigen::Vector3f bary, hit;
    EXPECT_FALSE(nm.locate_navcel(p, s, cid, bary, &hit));
    EXPECT_TRUE(std::isnan(nm.sample_layer_at("missing", p)));
  }

  NavCelId cid; float t; Eigen::Vector3f hit;
  EXPECT_FALSE(nm.raycast({kNaN, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}, cid, t, hit));
  EXPECT_FALSE(nm.raycast({0.0f, 0.0f, 10.0f}, {kNaN, kNaN, kNaN}, cid, t, hit));
  EXPECT_FALSE(nm.raycast({0.0f, 0.0f, 10.0f}, {0.0f, 0.0f, 0.0f}, cid, t, hit));
}

TEST(NavMap_Robustness, EmptyMapAndEmptySurfaces)
{
  NavMap empty;
  empty.rebuild_geometry_accels();
  std::size_t s; NavCelId cid; Eigen::Vector3f bary, hit, cp; float t, sq;
  EXPECT_FALSE(empty.locate_navcel({0.0f, 0.0f, 0.0f}, s, cid, bary, &hit));
  EXPECT_FALSE(empty.raycast({0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f}, cid, t, hit));
  EXPECT_FALSE(empty.closest_navcel({0.0f, 0.0f, 0.0f}, s, cid, cp, sq));

  // Empty surfaces before and after a real one.
  NavMap nm;
  nm.create_surface("empty_a");
  const std::size_t real = nm.create_surface("map");
  nm.create_surface("empty_b");
  const auto a = nm.add_vertex({0.0f, 0.0f, 0.0f});
  const auto b = nm.add_vertex({1.0f, 0.0f, 0.0f});
  const auto c = nm.add_vertex({0.0f, 1.0f, 0.0f});
  nm.add_navcel_to_surface(real, nm.add_navcel(a, b, c));
  nm.rebuild_geometry_accels();
  ASSERT_TRUE(nm.locate_navcel({0.2f, 0.2f, 0.0f}, s, cid, bary, &hit));
  EXPECT_EQ(s, real);
  ASSERT_TRUE(nm.closest_navcel({5.0f, 5.0f, 0.0f}, s, cid, cp, sq));
  EXPECT_EQ(s, real);
}

TEST(NavMap_Robustness, DegenerateTrianglesAreIgnored)
{
  NavMap nm;
  const std::size_t s0 = nm.create_surface("map");
  const auto a = nm.add_vertex({0.0f, 0.0f, 0.0f});
  const auto b = nm.add_vertex({1.0f, 0.0f, 0.0f});
  const auto c = nm.add_vertex({0.0f, 1.0f, 0.0f});
  const auto d = nm.add_vertex({2.0f, 0.0f, 0.0f});
  nm.add_navcel_to_surface(s0, nm.add_navcel(a, a, b));  // repeated vertex
  nm.add_navcel_to_surface(s0, nm.add_navcel(a, b, d));  // collinear
  const NavCelId good = nm.add_navcel(a, b, c);
  nm.add_navcel_to_surface(s0, good);
  nm.rebuild_geometry_accels();

  for (const auto & cel : nm.navcels) {
    EXPECT_TRUE(cel.normal.allFinite());
    EXPECT_TRUE(std::isfinite(cel.area));
  }
  std::size_t s; NavCelId cid; Eigen::Vector3f bary, hit;
  ASSERT_TRUE(nm.locate_navcel({0.2f, 0.2f, 0.5f}, s, cid, bary, &hit));
  EXPECT_EQ(cid, good);
}

TEST(NavMap_Robustness, QueriesOnVerticesAndEdges)
{
  std::mt19937 rng(11);
  const NavMap nm = random_map(rng, 3);
  for (NavCelId cid = 0; cid < nm.navcels.size(); cid += 7) {
    const Eigen::Vector3f pa = vtx(nm, cid, 0), pb = vtx(nm, cid, 1);
    for (const Eigen::Vector3f & p : {pa, Eigen::Vector3f(0.5f * (pa + pb))}) {
      std::size_t s; NavCelId c; Eigen::Vector3f bary, hit;
      ASSERT_TRUE(nm.locate_navcel(p, s, c, bary, &hit));
      EXPECT_NEAR(hit.z(), p.z(), 2e-4f);
      EXPECT_EQ(nm.surface_of(c), s);
    }
  }
}

TEST(NavMap_Robustness, EveryCellAsHintGivesTheSameFloor)
{
  NavMap nm;
  for (float z : {0.0f, 3.0f}) {
    const std::size_t s = nm.create_surface("map");
    const auto b = static_cast<uint32_t>(nm.positions.size());
    nm.add_vertex({0, 0, z}); nm.add_vertex({4, 0, z});
    nm.add_vertex({4, 4, z}); nm.add_vertex({0, 4, z});
    nm.add_navcel_to_surface(s, nm.add_navcel(b, b + 1, b + 2));
    nm.add_navcel_to_surface(s, nm.add_navcel(b, b + 2, b + 3));
  }
  nm.rebuild_geometry_accels();

  for (float z : {0.0f, 0.2f, 3.0f, 3.2f}) {
    const std::size_t expected = z < 1.5f ? 0u : 1u;
    for (NavCelId h = 0; h < nm.navcels.size(); ++h) {
      NavMap::LocateOpts opts;
      opts.hint_cid = h;
      std::size_t s; NavCelId c; Eigen::Vector3f bary, hit;
      ASSERT_TRUE(nm.locate_navcel({1.0f, 2.5f, z}, s, c, bary, &hit, opts));
      EXPECT_EQ(s, expected) << "z=" << z << " hint=" << h;
    }
  }
}

TEST(NavMap_Robustness, CellInTwoSurfacesReportsTheFirstOwner)
{
  NavMap nm;
  const std::size_t s0 = nm.create_surface("a");
  const std::size_t s1 = nm.create_surface("b");
  const auto a = nm.add_vertex({0.0f, 0.0f, 0.0f});
  const auto b = nm.add_vertex({1.0f, 0.0f, 0.0f});
  const auto c = nm.add_vertex({0.0f, 1.0f, 0.0f});
  const NavCelId cid = nm.add_navcel(a, b, c);
  nm.add_navcel_to_surface(s0, cid);
  nm.add_navcel_to_surface(s1, cid);
  EXPECT_EQ(nm.surface_of(cid), s0);  // linear fallback (no rebuild yet)
  nm.rebuild_geometry_accels();
  EXPECT_EQ(nm.surface_of(cid), s0);  // index
}

TEST(NavMap_Robustness, RebuildIsIdempotentAndTracksEdits)
{
  std::mt19937 rng(21);
  NavMap nm = random_map(rng, 2);
  const auto before = nm.navcels.size();
  nm.rebuild_geometry_accels();
  nm.rebuild_geometry_accels();
  for (NavCelId cid = 0; cid < before; ++cid) {
    for (std::size_t s = 0; s < nm.surfaces.size(); ++s) {
      const auto & ids = nm.surfaces[s].navcels;
      if (std::find(ids.begin(), ids.end(), cid) != ids.end()) {
        ASSERT_EQ(nm.surface_of(cid), s);
      }
    }
  }

  // Remove the first surface and rebuild: indices shift accordingly.
  const auto moved = nm.surfaces[1].navcels.front();
  nm.remove_surface(0);
  nm.rebuild_geometry_accels();
  EXPECT_EQ(nm.surface_of(moved), 0u);
  EXPECT_EQ(nm.surface_of(nm.surfaces.size() > 0 ? nm.surfaces[0].navcels.back() : 0), 0u);
}
