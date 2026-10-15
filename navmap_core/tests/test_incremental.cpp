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
#include <vector>

#include "navmap_core/DistanceField.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_core/Occupancy.hpp"

using navmap::DistanceField;
using navmap::NavCelId;
using navmap::NavMap;

namespace
{

constexpr float kInf = std::numeric_limits<float>::infinity();

void add_grid(
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
}

std::vector<NavMap> maps()
{
  std::vector<NavMap> out(3);
  add_grid(out[0], 20, 20, 0.5f, [](float, float) {return 0.0f;},
    [](int i, int j) {
      return (i == 8 && j > 3 && j < 15) || (i > 12 && i < 15 && j > 12 && j < 15);
      });
  add_grid(out[1], 16, 16, 0.5f, [](float, float) {return 0.0f;});
  add_grid(out[1], 16, 16, 0.5f, [](float, float) {return 3.0f;});
  add_grid(out[2], 30, 6, 0.4f, [](float x, float) {return 0.25f * x;});
  for (auto & m : out) {
    m.rebuild_geometry_accels();
  }
  return out;
}

float max_diff(const std::vector<float> & a, const std::vector<float> & b)
{
  float m = 0.0f;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (std::isinf(a[i]) || std::isinf(b[i])) {
      if (std::isinf(a[i]) != std::isinf(b[i])) {return kInf;}
      continue;
    }
    m = std::max(m, std::abs(a[i] - b[i]));
  }
  return m;
}

}  // namespace

// ------------------------------------------------------------------ distance field

// Exact field by brute force: for every seed, the cells it reaches through adjacency within
// max_distance of it, at their straight distance.
std::vector<float> brute(
  const NavMap & nm, const std::vector<NavCelId> & seeds, bool boundary,
  float maxd)
{
  struct S {Eigen::Vector3f a, b; NavCelId cell;};
  std::vector<S> all;
  for (NavCelId c : seeds) {
    all.push_back({nm.navcel_centroid(c), nm.navcel_centroid(c), c});
  }
  if (boundary) {
    for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
      for (int e = 0; e < 3; ++e) {
        if (nm.navcels[c].neighbor[e] != std::numeric_limits<NavCelId>::max()) {continue;}
        all.push_back({nm.positions.at(nm.navcels[c].v[e]),
            nm.positions.at(nm.navcels[c].v[(e + 1) % 3]), c});
      }
    }
  }
  std::vector<float> d(nm.navcels.size(), kInf);
  for (const auto & sd : all) {
    auto dist = [&](NavCelId c) {
        return navmap::point_segment_distance(nm.navcel_centroid(c), sd.a, sd.b);
      };
    std::vector<char> seen(nm.navcels.size(), 0);
    std::vector<NavCelId> st{sd.cell};
    seen[sd.cell] = 1;
    while (!st.empty()) {
      const NavCelId c = st.back();
      st.pop_back();
      const float x = dist(c);
      if (x > maxd && c != sd.cell) {continue;}
      if (x <= maxd) {d[c] = std::min(d[c], x);}
      for (NavCelId v : nm.navcels[c].neighbor) {
        if (v != std::numeric_limits<NavCelId>::max() && !seen[v]) {seen[v] = 1; st.push_back(v);}
      }
    }
  }
  return d;
}

TEST(DistanceFieldIncremental, ComputeIsExactWithABoundAndMatchesNavMapWithout)
{
  for (const auto & nm : maps()) {
    std::mt19937 rng(1);
    std::uniform_int_distribution<NavCelId> C(0, static_cast<NavCelId>(nm.navcels.size() - 1));
    std::vector<NavCelId> seeds;
    for (int k = 0; k < 6; ++k) {
      seeds.push_back(C(rng));
    }
    for (const bool boundary : {false, true}) {
      DistanceField bounded(nm, boundary, 2.0f);
      bounded.compute(seeds);
      EXPECT_EQ(max_diff(bounded.distances(), brute(nm, seeds, boundary, 2.0f)), 0.0f);
      DistanceField unbounded(nm, boundary);
      unbounded.compute(seeds);
      EXPECT_EQ(unbounded.distances(), nm.distance_field(seeds, boundary));
      for (NavCelId s : seeds) {
        EXPECT_TRUE(bounded.is_seed(s) && unbounded.is_seed(s));
      }
    }
  }
}

TEST(DistanceFieldIncremental, RandomUpdatesMatchARecomputationAndReportTheChanges)
{
  int m = 0;
  for (const auto & nm : maps()) {
    for (const bool boundary : {false, true}) {
      for (const float maxd : {kInf, 1.5f}) {
        std::mt19937 rng(10 + m++);
        std::uniform_int_distribution<NavCelId> C(0, static_cast<NavCelId>(nm.navcels.size() - 1));
        std::set<NavCelId> seeds;
        DistanceField df(nm, boundary, maxd);
        df.compute({});
        float worst = 0.0f;
        for (int step = 0; step < 60; ++step) {
          std::vector<NavCelId> add, rem;
          const int na = static_cast<int>(rng() % 4),
            nr = seeds.empty() ? 0 : static_cast<int>(rng() % 3);
          for (int k = 0; k < na; ++k) {
            add.push_back(C(rng));
          }
          for (int k = 0; k < nr; ++k) {
            auto it = seeds.begin();
            std::advance(it, rng() % seeds.size());
            rem.push_back(*it);
          }
          if (rng() % 5 == 0) {rem.push_back(C(rng));}  // removing a non-seed: ignored
          if (rng() % 5 == 0 && !add.empty()) {add.push_back(add.front());}  // repeated
          const std::vector<float> prev = df.distances();
          df.update(add, rem);
          for (NavCelId r : rem) {
            seeds.erase(r);
          }
          for (NavCelId a : add) {
            seeds.insert(a);
          }
          DistanceField ref(nm, boundary, maxd);
          ref.compute({seeds.begin(), seeds.end()});
          const float diff = max_diff(df.distances(), ref.distances());
          if (diff > 1e-5f && worst <= 1e-5f) {
            for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
              if (std::abs(df.distances()[c] - ref.distances()[c]) > 1e-5f) {
                const auto p = nm.navcel_centroid(c);
                ADD_FAILURE() << "step " << step << " cell " << c << " at " << p.transpose() <<
                  " inc "
                              << df.distances()[c] << " full " << ref.distances()[c];
                break;
              }
            }
          }
          worst = std::max(worst, diff);
          // changed() is exactly the set of cells whose value differs
          std::set<NavCelId> want;
          for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
            const float a = prev[c], b = df.distances()[c];
            if (!(a == b)) {want.insert(c);}
          }
          const std::set<NavCelId> got(df.changed().begin(), df.changed().end());
          ASSERT_EQ(got, want) << "step " << step;
          EXPECT_EQ(got.size(), df.changed().size());  // no duplicates
          for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
            ASSERT_EQ(df.is_seed(c), seeds.count(c) > 0);
          }
        }
        EXPECT_LE(worst, 1e-5f) << "map " << m - 1 << " boundary " << boundary << " max " << maxd;
      }
    }
  }
}

TEST(DistanceFieldIncremental, AddingAndRemovingTheSameCellAndEdgeIds)
{
  const NavMap nm = maps()[1];
  DistanceField df(nm, false, 2.0f);
  df.compute({5});
  // Same cell removed and added in one call: removal first, so it stays a seed
  df.update({5}, {5});
  EXPECT_TRUE(df.is_seed(5));
  EXPECT_EQ(df[5], 0.0f);
  // Out-of-range ids are ignored
  df.update({99999}, {99999});
  EXPECT_FALSE(df.is_seed(99999));
  // Removing the only seed: everything unreachable again
  df.update({}, {5});
  for (float d : df.distances()) {
    EXPECT_TRUE(std::isinf(d));
                                                             }
  // A seed on the upper floor never reaches the lower one
  NavCelId upper = 0;
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    if (nm.navcel_centroid(c).z() > 2.0f) {upper = c; break;}
  }
  df.update({upper}, {});
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    if (nm.navcel_centroid(c).z() < 1.0f) {EXPECT_TRUE(std::isinf(df[c]));}
  }
}

TEST(DistanceFieldIncremental, AnUpdateOnlyTouchesItsNeighbourhood)
{
  // A large map: adding one seed changes only the cells within max_distance of it.
  NavMap nm;
  add_grid(nm, 200, 200, 0.1f, [](float, float) {return 0.0f;});
  nm.rebuild_geometry_accels();
  DistanceField df(nm, true, 0.5f);
  df.compute({});
  std::size_t s;
  NavCelId c;
  Eigen::Vector3f bary, hit;
  ASSERT_TRUE(nm.locate_navcel({10.03f, 10.07f, 0.0f}, s, c, bary, &hit));
  df.update({c}, {});
  EXPECT_GT(df.changed().size(), 100u);
  EXPECT_LT(df.changed().size(), 2u * 3.2f * 25.0f * 2.0f);  // ~ pi r^2 / (res^2 / 2), with margin
  for (NavCelId x : df.changed()) {
    EXPECT_LE((nm.navcel_centroid(x) - nm.navcel_centroid(c)).norm(), 0.5f + 1e-4f);
  }
  df.update({}, {c});
  EXPECT_EQ(df.changed().size(), df.changed().size());
  for (NavCelId x = 0; x < nm.navcels.size(); ++x) {
    ASSERT_TRUE(std::isinf(df[x]) || df[x] <= 0.5f);
  }
}

// ------------------------------------------------------------------ layer change tracking

TEST(LayerChanges, SingleWritesAreRecordedOnceAndBulkWritesMarkTheLayer)
{
  NavMap nm = maps()[1];
  auto v = nm.add_layer<uint8_t>("occ", "", "", uint8_t{0});
  EXPECT_TRUE(v->all_changed());  // new layer: all new
  v->clear_changes();
  EXPECT_FALSE(v->has_changes());
  nm.layer_set<uint8_t>("occ", 7, 254);
  nm.layer_set<uint8_t>("occ", 3, 254);
  nm.layer_set<uint8_t>("occ", 7, 100);  // again: recorded once
  EXPECT_FALSE(v->all_changed());
  EXPECT_EQ(v->changed(), (std::vector<NavCelId>{7, 3}));
  v->clear_changes();
  EXPECT_TRUE(v->changed().empty());
  nm.layer_set<uint8_t>("occ", 7, 1);  // recorded again after a clear
  EXPECT_EQ(v->changed(), (std::vector<NavCelId>{7}));
  nm.layer_clear<uint8_t>("occ", 0);
  EXPECT_TRUE(v->all_changed());
  v->clear_changes();
  v->set_data(std::vector<uint8_t>(nm.navcels.size(), 3));
  EXPECT_TRUE(v->all_changed());
  v->clear_changes();
  // Out-of-range writes record nothing; mark_changed() for direct writes through data()
  nm.layer_set<uint8_t>("occ", 99999, 1);
  EXPECT_FALSE(v->has_changes());
  v->data()[11] = 9;
  v->mark_changed(11);
  EXPECT_EQ(v->changed(), (std::vector<NavCelId>{11}));
}

TEST(LayerChanges, HashFollowsChangesAndCopiesCarryPendingOnes)
{
  NavMap nm = maps()[0];
  auto v = nm.add_layer<float>("f", "", "", 0.0f);
  v->clear_changes();
  const auto h0 = v->content_hash();
  nm.layer_set<float>("f", 2, 1.0f);
  EXPECT_NE(v->content_hash(), h0);
  NavMap copy(nm);
  auto cv = copy.layers.get("f");
  EXPECT_EQ(cv->changed(), (std::vector<NavCelId>{2}));
  cv->clear_changes();  // independent from the original's
  EXPECT_EQ(v->changed(), (std::vector<NavCelId>{2}));
}

TEST(LayerChanges, OccupancyUpdatesAreRecorded)
{
  NavMap nm = maps()[1];
  navmap::OccupancyLayer occ(nm);
  occ.view()->clear_changes();
  occ.hit(4);
  occ.miss(9);
  occ.hit(4);
  occ.reset(12);
  EXPECT_EQ(occ.view()->changed(), (std::vector<NavCelId>{4, 9, 12}));
}
