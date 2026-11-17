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

#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include "navmap_core/NavMap.hpp"
#include "navmap_core/Subdivide.hpp"

using navmap::NavCelId;
using navmap::NavMap;

namespace
{

// A grid of n x m cells of side `cell`, two NavCels each, height z(x, y), and a layer "column"
// with the column of each cell
NavMap grid(int n, int m, float cell, const std::function<float(float, float)> & z)
{
  NavMap nm;
  const auto s = nm.create_surface("map");
  for (int j = 0; j <= m; ++j) {
    for (int i = 0; i <= n; ++i) {
      nm.positions.x.push_back(i * cell);
      nm.positions.y.push_back(j * cell);
      nm.positions.z.push_back(z(i * cell, j * cell));
    }
  }
  std::vector<uint8_t> column;
  for (int j = 0; j < m; ++j) {
    for (int i = 0; i < n; ++i) {
      const uint32_t a = j * (n + 1) + i, b = a + 1, c = a + n + 1, d = c + 1;
      nm.add_navcel_to_surface(s, nm.add_navcel(a, b, d));
      nm.add_navcel_to_surface(s, nm.add_navcel(a, d, c));
      column.push_back(static_cast<uint8_t>(i));
      column.push_back(static_cast<uint8_t>(i));
    }
  }
  auto layer = nm.add_layer<uint8_t>("column");
  layer->set_data(column);
  nm.rebuild_geometry_accels();
  return nm;
}

float flat(float, float) {return 0.0f;}

double area(const NavMap & nm)
{
  double a = 0.0;
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    const auto p = nm.positions.at(nm.navcels[c].v[0]), q = nm.positions.at(nm.navcels[c].v[1]),
      r = nm.positions.at(nm.navcels[c].v[2]);
    a += 0.5 * (q - p).cross(r - p).norm();
  }
  return a;
}

// Conforming: no vertex in the middle of an edge (a crack: the NavCels on both sides of the edge
// would not share it whole)
int t_junctions(const NavMap & nm)
{
  std::map<std::pair<uint32_t, uint32_t>, int> edges;
  for (const auto & c : nm.navcels) {
    for (int k = 0; k < 3; ++k) {
      const uint32_t a = c.v[k], b = c.v[(k + 1) % 3];
      edges[{std::min(a, b), std::max(a, b)}]++;
    }
  }
  int bad = 0;
  for (const auto & [e, n] : edges) {
    const Eigen::Vector3f a = nm.positions.at(e.first), b = nm.positions.at(e.second);
    for (uint32_t v = 0; v < nm.positions.size(); ++v) {
      if (v == e.first || v == e.second) {continue;}
      const Eigen::Vector3f p = nm.positions.at(v);
      const float t = (p - a).dot(b - a) / (b - a).squaredNorm();
      if (t > 1e-4f && t < 1.0f - 1e-4f && (a + t * (b - a) - p).norm() < 1e-4f) {++bad;}
    }
  }
  return bad;
}

float longest_edge(const NavMap & nm, NavCelId c)
{
  float l = 0.0f;
  for (int k = 0; k < 3; ++k) {
    l = std::max(
      l, (nm.positions.at(nm.navcels[c].v[k]) -
      nm.positions.at(nm.navcels[c].v[(k + 1) % 3])).norm());
  }
  return l;
}

}  // namespace

TEST(Subdivide, NoNavCelCrossesTheLinesAfterTheCut)
{
  NavMap nm = grid(10, 10, 1.0f, flat);
  const double a0 = area(nm);
  const auto st = navmap::cut_along_lines(nm, {{2.5f, 5.5f}, {3.3f}});
  EXPECT_GT(st.navcels_after, st.navcels_before);
  EXPECT_NEAR(area(nm), a0, 1e-3);
  EXPECT_EQ(t_junctions(nm), 0);
  for (const auto & c : nm.navcels) {
    for (const float x : {2.5f, 5.5f}) {
      bool below = false, above = false;
      for (const auto v : c.v) {
        below |= nm.positions.x[v] < x;
        above |= nm.positions.x[v] > x;
      }
      EXPECT_FALSE(below && above);
    }
    bool below = false, above = false;
    for (const auto v : c.v) {
      below |= nm.positions.y[v] < 3.3f;
      above |= nm.positions.y[v] > 3.3f;
    }
    EXPECT_FALSE(below && above);
  }
  // the pieces keep the values of the NavCel they come from, and the surface has them all
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    EXPECT_EQ(
      nm.layer_get<uint8_t>("column", c),
      static_cast<uint8_t>(std::floor(nm.navcel_centroid(c).x())));
  }
  EXPECT_EQ(nm.surfaces[0].navcels.size(), nm.navcels.size());
}

TEST(Subdivide, VerticesOnALineHaveItsCoordinateExactly)
{
  // a line 0.00005 m from a column of vertices: they go onto it, no sliver is made
  NavMap nm = grid(4, 4, 1.0f, [](float x, float y) {return 0.5f * x + 0.25f * y;});
  navmap::cut_along_lines(nm, {{2.00005f, 1.7f}, {}});
  int on = 0;
  for (std::size_t v = 0; v < nm.positions.size(); ++v) {
    const float x = nm.positions.x[v];
    if (std::abs(x - 1.7f) < 1e-3f) {EXPECT_EQ(x, 1.7f); ++on;}
    if (std::abs(x - 2.00005f) < 1e-3f) {EXPECT_EQ(x, 2.00005f);}
    // on the plane (a vertex moved onto a line keeps its height: within the snap times the slope)
    EXPECT_NEAR(nm.positions.z[v], 0.5f * x + 0.25f * nm.positions.y[v], 1e-4f);
  }
  EXPECT_GT(on, 4);
  EXPECT_EQ(t_junctions(nm), 0);
}

TEST(Subdivide, TheRefinementIsWhereThePointsAre)
{
  NavMap nm = grid(1, 1, 16.0f, flat);   // two NavCels of 16 m legs
  const double a0 = area(nm);
  navmap::refine(nm, {Eigen::Vector3f(3.0f, 2.0f, 0.0f)}, 1.0f);
  EXPECT_NEAR(area(nm), a0, 1e-3);
  EXPECT_EQ(t_junctions(nm), 0);
  std::size_t s;
  NavCelId c;
  Eigen::Vector3f bary;
  ASSERT_TRUE(nm.locate_navcel(Eigen::Vector3f(3.0f, 2.0f, 0.0f), s, c, bary, nullptr));
  EXPECT_LE(longest_edge(nm, c), 1.0f + 1e-4f);
  ASSERT_TRUE(nm.locate_navcel(Eigen::Vector3f(14.0f, 15.0f, 0.0f), s, c, bary, nullptr));
  EXPECT_GT(longest_edge(nm, c), 4.0f) << "far from the point the NavCels stay large";
  EXPECT_LT(nm.navcels.size(), 200u);
  EXPECT_EQ(nm.surfaces[0].navcels.size(), nm.navcels.size());
}

TEST(Subdivide, RefiningCarriesTheLayersAndIgnoresPointsOffTheMap)
{
  NavMap nm = grid(4, 1, 4.0f, flat);
  NavMap same = nm;
  navmap::refine(same, {Eigen::Vector3f(-10.0f, -10.0f, 0.0f)}, 0.5f);
  EXPECT_EQ(same.navcels.size(), nm.navcels.size());

  navmap::refine(nm, {Eigen::Vector3f(9.0f, 1.0f, 0.0f), Eigen::Vector3f(1.0f, 3.0f, 0.0f)}, 0.5f);
  EXPECT_EQ(t_junctions(nm), 0);
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    EXPECT_EQ(
      nm.layer_get<uint8_t>("column", c),
      static_cast<uint8_t>(std::floor(nm.navcel_centroid(c).x() / 4.0f)));
  }
}

TEST(Subdivide, NavCelsAtTheEndOfALinkAreNotSplit)
{
  NavMap nm = grid(2, 1, 8.0f, flat);
  nm.add_link(0, 3, navmap::LinkType::GENERIC, 1.0f, true);
  navmap::refine(nm, {Eigen::Vector3f(1.0f, 4.0f, 0.0f)}, 0.5f);
  EXPECT_EQ(t_junctions(nm), 0);
  ASSERT_EQ(nm.links().size(), 1u);
  const auto & l = nm.links()[0];
  EXPECT_GT(longest_edge(nm, l.from), 8.0f);
  EXPECT_GT(longest_edge(nm, l.to), 8.0f);
}

TEST(Subdivide, APointOnAnEdgeRefinesTheNavCelsOnBothSides)
{
  // (8, 8) is on the diagonal of the square: in both NavCels; then on edges of the splits too
  NavMap nm = grid(1, 1, 16.0f, flat);
  navmap::refine(nm, {Eigen::Vector3f(8.0f, 8.0f, 0.0f), Eigen::Vector3f(4.0f, 0.0f, 0.0f)}, 1.0f);
  EXPECT_EQ(t_junctions(nm), 0);
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    for (const auto & q : {Eigen::Vector2f(8.0f, 8.0f), Eigen::Vector2f(4.0f, 0.0f)}) {
      // every NavCel with the point inside or on its border
      const Eigen::Vector2f a = nm.positions.at(nm.navcels[c].v[0]).head<2>();
      const Eigen::Vector2f b = nm.positions.at(nm.navcels[c].v[1]).head<2>();
      const Eigen::Vector2f d = nm.positions.at(nm.navcels[c].v[2]).head<2>();
      auto cross = [](Eigen::Vector2f u, Eigen::Vector2f v) {return u.x() * v.y() - u.y() * v.x();};
      const float s1 = cross(b - a, q - a), s2 = cross(d - b, q - b), s3 = cross(a - d, q - d);
      const bool in = (s1 >= -1e-5f && s2 >= -1e-5f && s3 >= -1e-5f) ||
        (s1 <= 1e-5f && s2 <= 1e-5f && s3 <= 1e-5f);
      if (in) {EXPECT_LE(longest_edge(nm, c), 1.0f + 1e-4f) << q.transpose();}
    }
  }
}

TEST(Subdivide, ARegionIsRefinedAsFineAsItsBoundSays)
{
  // every NavCel within 2 m of (8, 8) no larger than 1 m; the rest as it is
  NavMap nm = grid(2, 2, 8.0f, flat);
  const Eigen::Vector2f p(8.0f, 8.0f);
  auto near = [&](const Eigen::Vector3f & a, const Eigen::Vector3f & b, const Eigen::Vector3f & c) {
      const Eigen::Vector2f q = ((a + b + c) / 3.0f).head<2>();
      float reach = 0.0f;
      for (const auto * v : {&a, &b, &c}) {
        reach = std::max(reach, (v->head<2>() - q).norm());
      }
      return (q - p).norm() < 2.0f + reach ? 1.0f : 0.0f;
    };
  const auto st = navmap::refine(nm, near);
  EXPECT_GT(st.navcels_after, st.navcels_before);
  EXPECT_EQ(t_junctions(nm), 0);
  bool large = false;
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    const Eigen::Vector2f q = nm.navcel_centroid(c).head<2>();
    if ((q - p).norm() < 2.0f) {EXPECT_LE(longest_edge(nm, c), 1.0f + 1e-4f);}
    large |= longest_edge(nm, c) > 4.0f;
  }
  EXPECT_TRUE(large) << "far from the point the NavCels stay large";
}
