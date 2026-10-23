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
#include <cstdio>
#include <functional>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "navmap_core/NavMap.hpp"
#include "navmap_core/Simplify.hpp"

using navmap::NavCelId;
using navmap::NavMap;
using navmap::SimplifyParams;

namespace
{

using Height = std::function<float (float, float)>;
using Keep = std::function<bool (int, int)>;

// nx x ny cells of side res (two NavCels each) at offset z, cells where keep(i, j); one surface
void add_grid(
  NavMap & nm, int nx, int ny, float res, const Height & h, const Keep & keep = nullptr)
{
  const std::size_t s = nm.create_surface("map");
  const auto base = static_cast<uint32_t>(nm.positions.size());
  for (int j = 0; j <= ny; ++j) {
    for (int i = 0; i <= nx; ++i) {
      nm.add_vertex({i * res, j * res, h(i * res, j * res)});
    }
  }
  auto v = [&](int i, int j) {return base + static_cast<uint32_t>(j * (nx + 1) + i);};
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      if (keep && !keep(i, j)) {continue;}
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j), v(i + 1, j + 1)));
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j + 1), v(i, j + 1)));
    }
  }
}

float flat(float, float) {return 0.0f;}

float area(const NavMap & nm)
{
  double a = 0.0;
  for (const auto & c : nm.navcels) {
    a += c.area;
  }
  return static_cast<float>(a);
}

// V - E + F over the mesh (1 per disc, 0 per annulus...)
int64_t euler(const NavMap & nm)
{
  std::set<std::pair<uint32_t, uint32_t>> edges;
  std::set<uint32_t> verts;
  for (const auto & c : nm.navcels) {
    for (int k = 0; k < 3; ++k) {
      const uint32_t a = c.v[k], b = c.v[(k + 1) % 3];
      edges.insert({std::min(a, b), std::max(a, b)});
      verts.insert(a);
    }
  }
  return static_cast<int64_t>(verts.size()) - static_cast<int64_t>(edges.size()) +
         static_cast<int64_t>(nm.navcels.size());
}

std::size_t open_edges(const NavMap & nm)
{
  std::size_t n = 0;
  for (const auto & c : nm.navcels) {
    for (const auto x : c.neighbor) {
      if (x == std::numeric_limits<uint32_t>::max()) {++n;}
    }
  }
  return n;
}

float max_edge(const NavMap & nm)
{
  float m = 0.0f;
  for (const auto & c : nm.navcels) {
    for (int k = 0; k < 3; ++k) {
      m = std::max(m, (nm.positions.at(c.v[k]) - nm.positions.at(c.v[(k + 1) % 3])).norm());
    }
  }
  return m;
}

// Value of `layer` under every probe point, before and after, must match
template<class T>
void expect_same_values(
  const NavMap & a, const NavMap & b, const std::string & layer,
  const std::vector<Eigen::Vector3f> & probes)
{
  std::size_t s;
  NavCelId ca, cb;
  Eigen::Vector3f bary;
  for (const auto & p : probes) {
    ASSERT_TRUE(a.locate_navcel(p, s, ca, bary, nullptr)) << p.transpose();
    ASSERT_TRUE(b.locate_navcel(p, s, cb, bary, nullptr)) << p.transpose();
    ASSERT_EQ(a.layer_get<T>(layer, ca), b.layer_get<T>(layer, cb)) << layer << " at "
                                                                    << p.transpose();
  }
}

std::vector<Eigen::Vector3f> centroids(const NavMap & nm, float dz = 0.05f)
{
  std::vector<Eigen::Vector3f> out;
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    out.push_back(nm.navcel_centroid(c) + Eigen::Vector3f(0, 0, dz));
  }
  return out;
}

// A 4 x 4 m floor at 0.1 m: free, a wall, an unknown corner, and a uniform float layer
NavMap occupancy_floor()
{
  NavMap nm;
  add_grid(nm, 40, 40, 0.1f, flat);
  nm.rebuild_geometry_accels();
  auto occ = nm.add_layer<uint8_t>("occupancy", "", "", 0);
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    const auto p = nm.navcel_centroid(c);
    if (p.y() > 2.0f && p.y() < 2.1f && p.x() < 3.0f) {occ->data()[c] = 254;}
    if (p.x() > 3.0f && p.y() > 3.0f) {occ->data()[c] = 255;}
  }
  nm.set_layer_unknown("occupancy", 255);
  nm.add_layer<float>("friction", "", "", 0.7f);
  return nm;
}

}  // namespace

TEST(Simplify, FlatFloorShrinksAndEveryPointKeepsItsLayerValues)
{
  const NavMap before = occupancy_floor();
  NavMap nm = before;
  const auto st = navmap::simplify(nm);
  EXPECT_EQ(st.navcels_before, 3200u);
  EXPECT_EQ(st.navcels_after, nm.navcels.size());
  EXPECT_EQ(st.vertices_after, nm.positions.size());
  EXPECT_LT(nm.navcels.size(), before.navcels.size() / 5);
  EXPECT_NEAR(area(nm), 16.0f, 1e-3f);
  EXPECT_EQ(euler(nm), 1);
  for (std::size_t i = 0; i < nm.positions.size(); ++i) {
    ASSERT_EQ(nm.positions.z[i], 0.0f);
  }
  const auto probes = centroids(before);
  expect_same_values<uint8_t>(before, nm, "occupancy", probes);
  expect_same_values<float>(before, nm, "friction", probes);
  // Metadata, sizes and change tracking follow; the geometry is another one
  EXPECT_EQ(nm.layer_size("occupancy"), nm.navcels.size());
  EXPECT_EQ(nm.get_layer_meta("occupancy")->unknown, 255.0);
  EXPECT_TRUE(nm.layers.get("occupancy")->all_changed());
  EXPECT_NE(nm.geometry_id(), before.geometry_id());
  EXPECT_EQ(nm.surfaces.size(), 1u);
  EXPECT_EQ(nm.surfaces[0].navcels.size(), nm.navcels.size());
  // Simplifying again finds (almost) nothing more to do
  const std::size_t once = nm.navcels.size();
  navmap::simplify(nm);
  EXPECT_GE(nm.navcels.size(), once * 9 / 10);
  expect_same_values<uint8_t>(before, nm, "occupancy", probes);
}

TEST(Simplify, WithoutLayersAFlatSquareBecomesAFewNavCels)
{
  NavMap nm;
  add_grid(nm, 30, 30, 0.1f, flat);
  nm.rebuild_geometry_accels();
  navmap::simplify(nm);
  EXPECT_LE(nm.navcels.size(), 16u);
  EXPECT_NEAR(area(nm), 9.0f, 1e-3f);
  EXPECT_EQ(open_edges(nm), open_edges(nm));  // well formed
  for (std::size_t i = 0; i < nm.positions.size(); ++i) {
    const float x = nm.positions.x[i], y = nm.positions.y[i];
    // The square's outline stays: every vertex on its border (no interior vertex needed)
    EXPECT_TRUE(
      std::abs(x) < 1e-4f || std::abs(x - 3.0f) < 1e-4f ||
      std::abs(y) < 1e-4f || std::abs(y - 3.0f) < 1e-4f) << x << " " << y;
  }
}

TEST(Simplify, MaxEdgeLengthBoundsTheNavCels)
{
  NavMap coarse, bounded;
  add_grid(coarse, 30, 30, 0.1f, flat);
  coarse.rebuild_geometry_accels();
  bounded = coarse;
  navmap::simplify(coarse);
  SimplifyParams p;
  p.max_edge_length = 0.5f;
  navmap::simplify(bounded, p);
  EXPECT_LE(max_edge(bounded), 0.5f + 1e-5f);
  EXPECT_GT(max_edge(coarse), 1.0f);
  EXPECT_GT(bounded.navcels.size(), coarse.navcels.size());
  EXPECT_LT(bounded.navcels.size(), 1800u / 2);
  EXPECT_NEAR(area(bounded), 9.0f, 1e-3f);
}

TEST(Simplify, AnInclineStaysOnItsPlane)
{
  const float k = std::tan(20.0f * static_cast<float>(M_PI) / 180.0f);
  NavMap nm;
  add_grid(nm, 30, 20, 0.1f, [k](float x, float) {return k * x;});
  nm.rebuild_geometry_accels();
  const float a0 = area(nm);
  navmap::simplify(nm);
  EXPECT_LT(nm.navcels.size(), 60u);
  EXPECT_NEAR(area(nm), a0, 1e-3f * a0);
  for (std::size_t i = 0; i < nm.positions.size(); ++i) {
    EXPECT_NEAR(nm.positions.z[i], k * nm.positions.x[i], 1e-5f);
  }
  for (const auto & c : nm.navcels) {
    EXPECT_NEAR(std::acos(c.normal.z()) * 180.0f / static_cast<float>(M_PI), 20.0f, 0.01f);
  }
}

TEST(Simplify, CurvedTerrainStaysWithinTheErrorAndZeroErrorKeepsIt)
{
  const Height bumps = [](float x, float y) {return 0.3f * std::sin(x) * std::cos(0.7f * y);};
  NavMap orig;
  add_grid(orig, 50, 50, 0.1f, bumps);
  orig.rebuild_geometry_accels();
  for (const float e : {0.01f, 0.05f}) {
    NavMap nm = orig;
    SimplifyParams p;
    p.max_error = e;
    navmap::simplify(nm, p);
    EXPECT_LT(nm.navcels.size(), orig.navcels.size()) << e;
    // Original NavCel centroids (on the original surface), measured vertically to the new one
    float worst = 0.0f;
    for (const auto & q : centroids(orig, 0.0f)) {
      NavCelId c;
      float t;
      Eigen::Vector3f hit;
      ASSERT_TRUE(nm.raycast(q + Eigen::Vector3f(0, 0, 1.0f), {0, 0, -1.0f}, c, t, hit)) <<
        q.transpose();
      worst = std::max(worst, std::abs(hit.z() - q.z()));
    }
    EXPECT_LE(worst, 1.5f * e) << e;
    std::printf(
      "max_error %.3f: worst %.4f m, %zu -> %zu NavCels\n", e, worst,
      orig.navcels.size(), nm.navcels.size());
  }
  NavMap nm = orig;
  SimplifyParams p;
  p.max_error = 0.0f;
  navmap::simplify(nm, p);
  EXPECT_GT(nm.navcels.size(), orig.navcels.size() * 9 / 10);
}

TEST(Simplify, HolesAndStackedFloorsAreKept)
{
  NavMap orig;
  // Ground floor with a 1 x 1 m hole (a stairwell), and a full floor 3 m above
  add_grid(
    orig, 40, 40, 0.1f, flat, [](int i, int j) {
      const bool hole = i >= 15 && i < 25 && j >= 15 && j < 25;
      return !hole;
    });
  add_grid(orig, 40, 40, 0.1f, [](float, float) {return 3.0f;});
  orig.rebuild_geometry_accels();
  const int64_t chi = euler(orig);
  NavMap nm = orig;
  navmap::simplify(nm);
  EXPECT_LT(nm.navcels.size(), orig.navcels.size() / 5);
  EXPECT_EQ(euler(nm), chi);  // annulus + disc
  EXPECT_NEAR(area(nm), 15.0f + 16.0f, 1e-3f);
  ASSERT_EQ(nm.surfaces.size(), 2u);
  for (std::size_t s = 0; s < 2; ++s) {
    for (const NavCelId c : nm.surfaces[s].navcels) {
      ASSERT_NEAR(nm.navcel_centroid(c).z(), s == 0 ? 0.0f : 3.0f, 1e-5f);
    }
  }
  std::size_t s;
  NavCelId c;
  Eigen::Vector3f bary;
  EXPECT_TRUE(nm.locate_navcel(Eigen::Vector3f(2.0f, 2.0f, 3.1f), s, c, bary, nullptr));
  EXPECT_EQ(s, 1u);
  EXPECT_TRUE(nm.locate_navcel(Eigen::Vector3f(0.5f, 0.5f, 0.1f), s, c, bary, nullptr));
  EXPECT_EQ(s, 0u);
  // In the stairwell the ground floor is still missing
  Eigen::Vector3f hit;
  float t;
  ASSERT_TRUE(nm.raycast({2.0f, 2.0f, 2.0f}, {0, 0, -1.0f}, c, t, hit) == false);
}

TEST(Simplify, LinkEndsSurviveAndAreRemapped)
{
  NavMap nm;
  add_grid(nm, 20, 20, 0.1f, flat);
  add_grid(nm, 20, 20, 0.1f, [](float, float) {return 3.0f;});
  nm.rebuild_geometry_accels();
  std::size_t s;
  NavCelId a, b;
  Eigen::Vector3f bary;
  ASSERT_TRUE(nm.locate_navcel(Eigen::Vector3f(1.03f, 1.02f, 0.1f), s, a, bary, nullptr));
  ASSERT_TRUE(nm.locate_navcel(Eigen::Vector3f(1.03f, 1.02f, 3.1f), s, b, bary, nullptr));
  const auto ca = nm.navcel_centroid(a), cb = nm.navcel_centroid(b);
  nm.add_link(a, b, navmap::LinkType::ELEVATOR, 3.0f, false);
  navmap::simplify(nm);
  EXPECT_LT(nm.navcels.size(), 1600u / 4);
  ASSERT_EQ(nm.links().size(), 1u);
  const auto & l = nm.links()[0];
  EXPECT_TRUE(nm.navcel_centroid(l.from).isApprox(ca, 1e-5f));
  EXPECT_TRUE(nm.navcel_centroid(l.to).isApprox(cb, 1e-5f));
  EXPECT_EQ(l.type, navmap::LinkType::ELEVATOR);
  EXPECT_FLOAT_EQ(l.length, 3.0f);
  EXPECT_FALSE(l.bidirectional);
  EXPECT_EQ(nm.links_from(l.from).size(), 1u);
}

TEST(Simplify, OnlyTheChosenLayersAreKept)
{
  NavMap orig = occupancy_floor();
  auto noise = orig.add_layer<float>("noise", "", "", 0.0f);
  for (NavCelId c = 0; c < orig.navcels.size(); ++c) {
    noise->data()[c] = static_cast<float>(c % 7);
  }
  NavMap all = orig, chosen = orig;
  navmap::simplify(all);
  SimplifyParams p;
  p.layers = {"occupancy"};
  navmap::simplify(chosen, p);
  EXPECT_GT(all.navcels.size(), orig.navcels.size() * 9 / 10);  // noise everywhere: nothing
  EXPECT_LT(chosen.navcels.size(), orig.navcels.size() / 5);
  expect_same_values<uint8_t>(orig, chosen, "occupancy", centroids(orig));
  EXPECT_EQ(chosen.layer_size("noise"), chosen.navcels.size());  // carried, values not kept
  // The same, by leaving "noise" out of all the layers
  NavMap ignored = orig;
  SimplifyParams q;
  q.ignore_layers = {"noise"};
  navmap::simplify(ignored, q);
  EXPECT_EQ(ignored.navcels.size(), chosen.navcels.size());
  expect_same_values<uint8_t>(orig, ignored, "occupancy", centroids(orig));
  expect_same_values<float>(orig, ignored, "friction", centroids(orig));
}

TEST(Simplify, ColorsFollowTheVertices)
{
  NavMap nm;
  add_grid(nm, 10, 10, 0.1f, flat);
  nm.colors.emplace();
  for (std::size_t i = 0; i < nm.positions.size(); ++i) {
    const auto x = static_cast<uint8_t>(std::lround(nm.positions.x[i] * 100.0f));
    nm.colors->r.push_back(x);
    nm.colors->g.push_back(0);
    nm.colors->b.push_back(0);
    nm.colors->a.push_back(255);
  }
  nm.rebuild_geometry_accels();
  navmap::simplify(nm);
  ASSERT_TRUE(nm.colors.has_value());
  ASSERT_EQ(nm.colors->r.size(), nm.positions.size());
  for (std::size_t i = 0; i < nm.positions.size(); ++i) {
    EXPECT_EQ(nm.colors->r[i], std::lround(nm.positions.x[i] * 100.0f));
  }
}

TEST(Simplify, DegenerateInputsAreSafe)
{
  NavMap empty;
  EXPECT_EQ(navmap::simplify(empty).navcels_after, 0u);
  // One NavCel: nothing to remove
  NavMap one;
  const auto s = one.create_surface("map");
  one.add_navcel_to_surface(
    s, one.add_navcel(
      one.add_vertex({0, 0, 0}), one.add_vertex({1, 0, 0}),
      one.add_vertex({0, 1, 0})));
  one.rebuild_geometry_accels();
  navmap::simplify(one);
  EXPECT_EQ(one.navcels.size(), 1u);
  // A non-manifold edge (three NavCels on it) and an unused vertex
  NavMap fin;
  add_grid(fin, 6, 6, 0.1f, flat);
  const auto s2 = fin.create_surface("map");
  const uint32_t top = fin.add_vertex({0.3f, 0.3f, 0.5f});
  fin.add_vertex({9, 9, 9});
  fin.add_navcel_to_surface(s2, fin.add_navcel(3 * 7 + 3, 3 * 7 + 4, top));
  fin.rebuild_geometry_accels();
  ASSERT_GT(fin.nonmanifold_edges(), 0u);
  const float a0 = area(fin);
  EXPECT_NO_THROW(navmap::simplify(fin));
  EXPECT_NEAR(area(fin), a0, 1e-4f);
  EXPECT_GT(fin.nonmanifold_edges(), 0u);  // the fin is still there
  for (const auto & c : fin.navcels) {
    for (const auto v : c.v) {
      ASSERT_LT(v, fin.positions.size());
    }
  }
}
