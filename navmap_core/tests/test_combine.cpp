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
#include <limits>
#include <string>
#include <vector>

#include "navmap_core/Combine.hpp"
#include "navmap_core/Costs.hpp"
#include "navmap_core/NavMap.hpp"

using navmap::CombineOp;
using navmap::CombineOptions;
using navmap::NavCelId;
using navmap::NavMap;

namespace
{

constexpr uint8_t U = navmap::NO_INFORMATION;
const float kNaN = std::numeric_limits<float>::quiet_NaN();

// A strip of n NavCels
NavMap strip(std::size_t n)
{
  NavMap nm;
  const std::size_t s = nm.create_surface("map");
  for (std::size_t i = 0; i <= n / 2 + 1; ++i) {
    nm.add_vertex({static_cast<float>(i), 0.0f, 0.0f});
    nm.add_vertex({static_cast<float>(i), 1.0f, 0.0f});
  }
  for (std::size_t k = 0; k < n; ++k) {
    const uint32_t i = static_cast<uint32_t>(k / 2) * 2;
    const NavCelId c = (k % 2 == 0) ? nm.add_navcel(i, i + 2, i + 3) : nm.add_navcel(
      i, i + 3,
      i + 1);
    nm.add_navcel_to_surface(s, c);
  }
  nm.rebuild_geometry_accels();
  return nm;
}

template<class T>
void set(NavMap & nm, const std::string & name, const std::vector<T> & v)
{
  auto l = nm.add_layer<T>(name);
  l->data() = v;
  l->clear_changes();
}

template<class T>
std::vector<T> get(const NavMap & nm, const std::string & name)
{
  std::vector<T> out;
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    out.push_back(nm.layer_get<T>(name, c));
  }
  return out;
}

// Combines s into d (both U8 cost layers with 255 unknown) and returns d
std::vector<uint8_t> cost(
  const std::vector<uint8_t> & d, const std::vector<uint8_t> & s, CombineOp op,
  CombineOptions o = {}, std::ptrdiff_t * changed = nullptr)
{
  NavMap nm = strip(d.size());
  set(nm, "d", d);
  set(nm, "s", s);
  nm.set_layer_unknown("d", U);
  nm.set_layer_unknown("s", U);
  const auto n = navmap::combine_layer(nm, "d", "s", op, o);
  if (changed) {*changed = n;}
  return get<uint8_t>(nm, "d");
}

}  // namespace

// Cells: (d, s) = (10, 50) (50, 10) (U, 30) (40, U) (U, U) (0, 0)
const std::vector<uint8_t> D = {10, 50, U, 40, U, 0};
const std::vector<uint8_t> S = {50, 10, 30, U, U, 0};

TEST(Combine, EachOperatorOnCostLayersWithUnknown)
{
  using V = std::vector<uint8_t>;
  EXPECT_EQ(cost(D, S, CombineOp::OVERWRITE), (V{50, 10, 30, 40, U, 0}));
  EXPECT_EQ(cost(D, S, CombineOp::TRUE_OVERWRITE), (V{50, 10, 30, U, U, 0}));
  EXPECT_EQ(cost(D, S, CombineOp::MAX), (V{50, 50, 30, 40, U, 0}));
  EXPECT_EQ(cost(D, S, CombineOp::MAX_KEEP_UNKNOWN), (V{50, 50, U, 40, U, 0}));
  EXPECT_EQ(cost(D, S, CombineOp::MIN), (V{10, 10, 30, 40, U, 0}));
  EXPECT_EQ(cost(D, S, CombineOp::ADD), (V{60, 60, 30, 40, U, 0}));
}

TEST(Combine, ReturnsTheChangedCellsAndMarksOnlyThem)
{
  std::ptrdiff_t n = -2;
  cost(D, S, CombineOp::MAX, {}, &n);
  EXPECT_EQ(n, 2);  // cells 0 and 2
  NavMap nm = strip(6);
  set(nm, "d", D);
  set(nm, "s", S);
  nm.set_layer_unknown("d", U);
  nm.set_layer_unknown("s", U);
  ASSERT_EQ(navmap::combine_layer(nm, "d", "s", CombineOp::MAX), 2);
  const auto l = nm.layers.get("d");
  EXPECT_FALSE(l->all_changed());
  auto ch = l->changed();
  std::sort(ch.begin(), ch.end());
  EXPECT_EQ(ch, (std::vector<NavCelId>{0, 2}));
  // Combining again changes nothing
  l->clear_changes();
  EXPECT_EQ(navmap::combine_layer(nm, "d", "s", CombineOp::MAX), 0);
  EXPECT_FALSE(l->has_changes());
}

TEST(Combine, AddSaturatesAtTheCapAndAtTheTypeRange)
{
  using V = std::vector<uint8_t>;
  CombineOptions o;
  o.cap = navmap::INSCRIBED_INFLATED_OBSTACLE;
  EXPECT_EQ(cost({200, 100, U}, {100, 100, 250}, CombineOp::ADD, o), (V{253, 200, 250}));
  // Without a cap the uint8 range clamps, and 255 would read as unknown: hence the cap
  EXPECT_EQ(cost({200}, {100}, CombineOp::ADD), (V{255}));
  // int16 layers clamp at their range too
  NavMap nm = strip(2);
  set<int16_t>(nm, "d", {32000, -32000});
  set<int16_t>(nm, "s", {1000, -1000});
  navmap::combine_layer(nm, "d", "s", CombineOp::ADD);
  EXPECT_EQ(get<int16_t>(nm, "d"), (std::vector<int16_t>{32767, -32768}));
}

TEST(Combine, OnlyTheGivenCells)
{
  CombineOptions o;
  o.cells = {1, 3, 99};  // out of range ids are ignored
  EXPECT_EQ(
    cost({0, 0, 0, 0}, {9, 9, 9, 9}, CombineOp::OVERWRITE, o),
    (std::vector<uint8_t>{0, 9, 0, 9}));
}

TEST(Combine, MixedTypesConvertAndUnknownsAreEachLayersOwn)
{
  NavMap nm = strip(5);
  // float source with NaN unknown into a U8 destination with 255 unknown
  set<float>(nm, "s", {3.6f, -2.0f, 400.0f, kNaN, 7.0f});
  nm.set_layer_unknown("s", kNaN);
  set<uint8_t>(nm, "d", {U, 1, 1, 1, 1});
  nm.set_layer_unknown("d", U);
  navmap::combine_layer(nm, "d", "s", CombineOp::OVERWRITE);
  EXPECT_EQ(get<uint8_t>(nm, "d"), (std::vector<uint8_t>{4, 0, 255, 1, 7}));
  // TRUE_OVERWRITE turns the source's unknown into the destination's unknown
  navmap::combine_layer(nm, "d", "s", CombineOp::TRUE_OVERWRITE);
  EXPECT_EQ(nm.layer_get<uint8_t>("d", 3), U);
  EXPECT_TRUE(nm.is_unknown("d", 3));
  // U8 into a float destination whose unknown is NaN: MAX replaces the unknown
  set<float>(nm, "f", {kNaN, 1.5f, 1.5f, kNaN, 300.0f});
  nm.set_layer_unknown("f", kNaN);
  set<uint8_t>(nm, "u", {7, 1, 2, U, 9});
  nm.set_layer_unknown("u", U);
  navmap::combine_layer(nm, "f", "u", CombineOp::MAX);
  const auto f = get<float>(nm, "f");
  EXPECT_FLOAT_EQ(f[0], 7.0f);
  EXPECT_FLOAT_EQ(f[1], 1.5f);
  EXPECT_FLOAT_EQ(f[2], 2.0f);
  EXPECT_TRUE(std::isnan(f[3]));  // both unknown
  EXPECT_FLOAT_EQ(f[4], 300.0f);
}

TEST(Combine, WithoutDeclaredUnknownsEveryValueCounts)
{
  NavMap nm = strip(3);
  set<uint8_t>(nm, "d", {255, 10, 255});
  set<uint8_t>(nm, "s", {10, 255, 3});
  navmap::combine_layer(nm, "d", "s", CombineOp::MIN);
  EXPECT_EQ(get<uint8_t>(nm, "d"), (std::vector<uint8_t>{10, 10, 3}));
  // TRUE_OVERWRITE of an unknown source into a destination without unknown: left as is
  nm.set_layer_unknown("s", 255);
  navmap::combine_layer(nm, "d", "s", CombineOp::TRUE_OVERWRITE);
  EXPECT_EQ(get<uint8_t>(nm, "d"), (std::vector<uint8_t>{10, 10, 3}));
  // NaN from a float source without declared unknown cannot go into an integer layer
  set<float>(nm, "nan", {kNaN, kNaN, 1.0f});
  navmap::combine_layer(nm, "d", "nan", CombineOp::OVERWRITE);
  EXPECT_EQ(get<uint8_t>(nm, "d"), (std::vector<uint8_t>{10, 10, 1}));
}

TEST(Combine, ProvenanceRecordsTheDecidingSource)
{
  NavMap nm = strip(4);
  set<uint8_t>(nm, "master", {0, 0, 0, 0});
  set<uint8_t>(nm, "prior", {0, 254, 0, 0});
  set<uint8_t>(nm, "obstacles", {254, 0, 0, 100});
  CombineOptions p;
  p.provenance = "who";
  p.source_id = 1;
  navmap::combine_layer(nm, "master", "prior", CombineOp::OVERWRITE, p);
  EXPECT_EQ(get<uint8_t>(nm, "who"), (std::vector<uint8_t>{1, 1, 1, 1}));
  EXPECT_EQ(nm.get_layer_meta("who")->description, "source that set each cell (0: none)");
  p.source_id = 2;
  navmap::combine_layer(nm, "master", "obstacles", CombineOp::MAX, p);
  EXPECT_EQ(get<uint8_t>(nm, "master"), (std::vector<uint8_t>{254, 254, 0, 100}));
  // Obstacles won cells 0 and 3, tied in cell 2 (0 == 0); the prior keeps cell 1
  EXPECT_EQ(get<uint8_t>(nm, "who"), (std::vector<uint8_t>{2, 1, 2, 2}));
}

TEST(Combine, APriorSurvivesClearingTheSensorLayer)
{
  // master = prior, then MAX obstacles. The sensor clears its own layer; the prior stays
  NavMap nm = strip(3);
  set<uint8_t>(nm, "prior", {254, 0, 0});
  set<uint8_t>(nm, "obstacles", {0, 254, 0});
  set<uint8_t>(nm, "master", {0, 0, 0});
  auto compose = [&]() {
      navmap::combine_layer(nm, "master", "prior", CombineOp::TRUE_OVERWRITE);
      navmap::combine_layer(nm, "master", "obstacles", CombineOp::MAX);
      return get<uint8_t>(nm, "master");
    };
  EXPECT_EQ(compose(), (std::vector<uint8_t>{254, 254, 0}));
  nm.layer_clear<uint8_t>("obstacles", 0);
  EXPECT_EQ(compose(), (std::vector<uint8_t>{254, 0, 0}));
}

TEST(Combine, InvalidRequestsAreRejected)
{
  NavMap nm = strip(3);
  set<uint8_t>(nm, "a", {1, 2, 3});
  set<uint8_t>(nm, "b", {1, 2, 3});
  set<float>(nm, "f", {1, 2, 3});
  EXPECT_EQ(navmap::combine_layer(nm, "a", "missing", CombineOp::MAX), -1);
  EXPECT_EQ(navmap::combine_layer(nm, "missing", "a", CombineOp::MAX), -1);
  EXPECT_EQ(navmap::combine_layer(nm, "a", "a", CombineOp::MAX), -1);
  CombineOptions o;
  o.provenance = "a";
  EXPECT_EQ(navmap::combine_layer(nm, "a", "b", CombineOp::MAX, o), -1);
  o.provenance = "f";  // exists with another type
  EXPECT_EQ(navmap::combine_layer(nm, "a", "b", CombineOp::MAX, o), -1);
  // A layer of another size (stale after the geometry changed)
  nm.layers.add_or_get<uint8_t>("short", 2, navmap::LayerType::U8);
  EXPECT_EQ(navmap::combine_layer(nm, "a", "short", CombineOp::MAX), -1);
  EXPECT_EQ(get<uint8_t>(nm, "a"), (std::vector<uint8_t>{1, 2, 3}));  // untouched
  // An empty map: nothing to do
  NavMap empty;
  empty.add_layer<uint8_t>("x");
  empty.add_layer<uint8_t>("y");
  EXPECT_EQ(navmap::combine_layer(empty, "x", "y", CombineOp::ADD), 0);
}
