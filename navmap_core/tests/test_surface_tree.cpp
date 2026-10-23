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

// Queries over many surfaces (through the BVH of surface boxes) give what a brute-force search
// over every triangle gives.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "navmap_core/NavMap.hpp"

using navmap::NavCelId;
using navmap::NavMap;

namespace
{

// Square patches of 2 x 2 NavCels (0.5 m) scattered over three overlapping floors, each its own
// surface; some tilted.
NavMap patches(int n, unsigned seed)
{
  NavMap nm;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> U(0.0f, 30.0f), T(-0.2f, 0.2f);
  std::uniform_int_distribution<int> F(0, 2);
  for (int k = 0; k < n; ++k) {
    const float x0 = U(rng), y0 = U(rng), z0 = 2.5f * F(rng), tilt = T(rng);
    const std::size_t s = nm.create_surface("map");
    const auto base = static_cast<uint32_t>(nm.positions.size());
    for (int j = 0; j <= 2; ++j) {
      for (int i = 0; i <= 2; ++i) {
        nm.add_vertex({x0 + 0.25f * i, y0 + 0.25f * j, z0 + tilt * 0.25f * i});
      }
    }
    auto v = [&](int i, int j) {return base + static_cast<uint32_t>(j * 3 + i);};
    for (int j = 0; j < 2; ++j) {
      for (int i = 0; i < 2; ++i) {
        nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j), v(i + 1, j + 1)));
        nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j + 1), v(i, j + 1)));
      }
    }
  }
  nm.rebuild_geometry_accels();
  return nm;
}

// Möller-Trumbore over every NavCel: nearest t
bool brute_raycast(
  const NavMap & nm, const Eigen::Vector3f & o, const Eigen::Vector3f & d,
  float & best)
{
  best = std::numeric_limits<float>::infinity();
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    const Eigen::Vector3f a = nm.positions.at(nm.navcels[c].v[0]);
    const Eigen::Vector3f b = nm.positions.at(nm.navcels[c].v[1]);
    const Eigen::Vector3f e = nm.positions.at(nm.navcels[c].v[2]);
    const Eigen::Vector3f e1 = b - a, e2 = e - a, p = d.cross(e2);
    const float det = e1.dot(p);
    if (std::fabs(det) < 1e-9f) {continue;}
    const Eigen::Vector3f tv = o - a;
    const float u = tv.dot(p) / det;
    const Eigen::Vector3f q = tv.cross(e1);
    const float w = d.dot(q) / det;
    const float t = e2.dot(q) / det;
    if (u < -1e-5f || w < -1e-5f || u + w > 1.0f + 1e-5f || t <= 0.0f) {continue;}
    best = std::min(best, t);
  }
  return std::isfinite(best);
}

// Closest point on a triangle (Ericson)
Eigen::Vector3f closest_on_tri(
  const Eigen::Vector3f & p, const Eigen::Vector3f & a,
  const Eigen::Vector3f & b, const Eigen::Vector3f & c)
{
  const Eigen::Vector3f ab = b - a, ac = c - a, ap = p - a;
  const float d1 = ab.dot(ap), d2 = ac.dot(ap);
  if (d1 <= 0 && d2 <= 0) {return a;}
  const Eigen::Vector3f bp = p - b;
  const float d3 = ab.dot(bp), d4 = ac.dot(bp);
  if (d3 >= 0 && d4 <= d3) {return b;}
  const float vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) {return a + d1 / (d1 - d3) * ab;}
  const Eigen::Vector3f cp = p - c;
  const float d5 = ab.dot(cp), d6 = ac.dot(cp);
  if (d6 >= 0 && d5 <= d6) {return c;}
  const float vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) {return a + d2 / (d2 - d6) * ac;}
  const float va = d3 * d6 - d5 * d4;
  if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
    return b + (d4 - d3) / ((d4 - d3) + (d5 - d6)) * (c - b);
  }
  const float den = 1.0f / (va + vb + vc);
  return a + ab * (vb * den) + ac * (vc * den);
}

float brute_closest_sq(const NavMap & nm, const Eigen::Vector3f & p)
{
  float best = std::numeric_limits<float>::infinity();
  for (const auto & cel : nm.navcels) {
    const Eigen::Vector3f q = closest_on_tri(
      p, nm.positions.at(cel.v[0]),
      nm.positions.at(cel.v[1]), nm.positions.at(cel.v[2]));
    best = std::min(best, (p - q).squaredNorm());
  }
  return best;
}

}  // namespace

TEST(SurfaceTree, RaycastMatchesBruteForce)
{
  const NavMap nm = patches(600, 1);
  std::mt19937 rng(2);
  std::uniform_real_distribution<float> U(-2.0f, 32.0f), Z(-1.0f, 7.0f);
  std::normal_distribution<float> G(0.0f, 1.0f);
  int hits = 0;
  for (int k = 0; k < 3000; ++k) {
    const Eigen::Vector3f o(U(rng), U(rng), Z(rng));
    // Half vertical (as locate), half random
    const Eigen::Vector3f d = k % 2 ? Eigen::Vector3f(0, 0, k % 4 == 1 ? -1.0f : 1.0f) :
      Eigen::Vector3f(G(rng), G(rng), G(rng)).normalized();
    float bt;
    const bool b = brute_raycast(nm, o, d, bt);
    NavCelId cid;
    float t;
    Eigen::Vector3f hit;
    const bool r = nm.raycast(o, d, cid, t, hit);
    ASSERT_EQ(r, b) << "ray " << k;
    if (r) {
      EXPECT_NEAR(t, bt, 1e-4f) << "ray " << k;
      ++hits;
    }
  }
  EXPECT_GT(hits, 100);
}

TEST(SurfaceTree, ClosestMatchesBruteForce)
{
  const NavMap nm = patches(600, 3);
  std::mt19937 rng(4);
  std::uniform_real_distribution<float> U(-5.0f, 35.0f), Z(-3.0f, 9.0f);
  for (int k = 0; k < 1000; ++k) {
    const Eigen::Vector3f p(U(rng), U(rng), Z(rng));
    std::size_t s;
    NavCelId cid;
    Eigen::Vector3f q;
    float sq;
    ASSERT_TRUE(nm.closest_navcel(p, s, cid, q, sq));
    EXPECT_NEAR(sq, brute_closest_sq(nm, p), 1e-4f) << "point " << k;
    EXPECT_EQ(nm.surface_of(cid), s);
  }
}

TEST(SurfaceTree, LocatePicksTheNearestFloorAmongManySurfaces)
{
  const NavMap nm = patches(600, 5);
  // Points just above NavCels, where patches of several surfaces may overlap
  std::mt19937 rng(6);
  std::uniform_int_distribution<NavCelId> C(0, static_cast<NavCelId>(nm.navcels.size() - 1));
  for (int k = 0; k < 2000; ++k) {
    const NavCelId target = C(rng);
    const Eigen::Vector3f p = nm.navcel_centroid(target) + Eigen::Vector3f(0, 0, 0.05f);
    std::size_t s;
    NavCelId cid;
    Eigen::Vector3f bary, hit;
    ASSERT_TRUE(nm.locate_navcel(p, s, cid, bary, &hit)) << "point " << k;
    // Brute force: of the NavCels over (x, y), the one closest in height
    float best = std::numeric_limits<float>::infinity();
    for (const auto & cel : nm.navcels) {
      const Eigen::Vector3f a = nm.positions.at(cel.v[0]), b = nm.positions.at(cel.v[1]);
      const Eigen::Vector3f c = nm.positions.at(cel.v[2]);
      const float den = (b.y() - c.y()) * (a.x() - c.x()) + (c.x() - b.x()) * (a.y() - c.y());
      const float l1 =
        ((b.y() - c.y()) * (p.x() - c.x()) + (c.x() - b.x()) * (p.y() - c.y())) / den;
      const float l2 =
        ((c.y() - a.y()) * (p.x() - c.x()) + (a.x() - c.x()) * (p.y() - c.y())) / den;
      if (l1 < -1e-4f || l2 < -1e-4f || 1.0f - l1 - l2 < -1e-4f) {continue;}
      const float z = l1 * a.z() + l2 * b.z() + (1.0f - l1 - l2) * c.z();
      if (std::fabs(z - p.z()) < std::fabs(best - p.z())) {best = z;}
    }
    EXPECT_NEAR(hit.z(), best, 1e-4f) << "point " << k;
  }
}

TEST(SurfaceTree, AddingASurfaceWithoutRebuildKeepsTheOthersAndARebuildFindsIt)
{
  NavMap nm = patches(50, 7);
  const Eigen::Vector3f above = nm.navcel_centroid(0) + Eigen::Vector3f(0, 0, 1.0f);
  NavCelId cid;
  float t;
  Eigen::Vector3f hit;
  ASSERT_TRUE(nm.raycast(above, {0, 0, -1}, cid, t, hit));
  // A new floor far from the others, the map's accelerators not rebuilt
  const std::size_t s = nm.create_surface("map");
  const auto a = nm.add_vertex({100.0f, 100.0f, 0.0f});
  const auto b = nm.add_vertex({101.0f, 100.0f, 0.0f});
  const auto c = nm.add_vertex({100.0f, 101.0f, 0.0f});
  nm.add_navcel_to_surface(s, nm.add_navcel(a, b, c));
  EXPECT_TRUE(nm.raycast(above, {0, 0, -1}, cid, t, hit));
  EXPECT_NEAR(t, 1.0f, 1e-4f);
  nm.rebuild_geometry_accels();
  EXPECT_TRUE(nm.raycast({100.2f, 100.2f, 1.0f}, {0, 0, -1}, cid, t, hit));
  EXPECT_NEAR(t, 1.0f, 1e-5f);
  std::size_t si;
  Eigen::Vector3f q;
  float sq;
  ASSERT_TRUE(nm.closest_navcel({100.2f, 100.2f, 0.5f}, si, cid, q, sq));
  EXPECT_EQ(si, s);
  EXPECT_NEAR(sq, 0.25f, 1e-5f);
}

TEST(SurfaceTree, EmptyAndSingleSurfaceMaps)
{
  NavMap empty;
  empty.rebuild_geometry_accels();
  NavCelId cid;
  float t;
  Eigen::Vector3f hit;
  EXPECT_FALSE(empty.raycast({0, 0, 1}, {0, 0, -1}, cid, t, hit));
  const NavMap one = patches(1, 9);
  std::size_t s;
  Eigen::Vector3f q;
  float sq;
  EXPECT_TRUE(one.closest_navcel({0, 0, 10}, s, cid, q, sq));
  EXPECT_NEAR(sq, brute_closest_sq(one, {0, 0, 10}), 1e-4f);
}
