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
#include <random>
#include <vector>

#include "navmap_core/NavMap.hpp"
#include "navmap_core/Occupancy.hpp"

using navmap::NavCelId;
using navmap::NavMap;
using navmap::OccupancyLayer;
using navmap::OccupancyParams;
using navmap::OccupancyState;

namespace
{

// n x n cells of 1 m, two triangles each.
NavMap grid(int n = 4)
{
  NavMap nm;
  const std::size_t s = nm.create_surface("map");
  for (int j = 0; j <= n; ++j) {
    for (int i = 0; i <= n; ++i) {
      nm.add_vertex({static_cast<float>(i), static_cast<float>(j), 0.0f});
    }
  }
  auto v = [&](int i, int j) {return static_cast<uint32_t>(j * (n + 1) + i);};
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j), v(i + 1, j + 1)));
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j + 1), v(i, j + 1)));
    }
  }
  nm.rebuild_geometry_accels();
  return nm;
}

}  // namespace

// ------------------------------------------------------------------ int16 layers

TEST(LayersI16, StoresSignedValuesAndConvertsBothWays)
{
  NavMap nm = grid();
  auto v = nm.add_layer<int16_t>("h", "height", "mm", int16_t{-5});
  EXPECT_EQ(nm.layer_type_name("h"), "int16");
  EXPECT_EQ(v->data().size(), nm.navcels.size());
  EXPECT_EQ(nm.layer_get<int16_t>("h", 0), -5);
  nm.layer_set<int16_t>("h", 3, -32768);
  nm.layer_set<int16_t>("h", 4, 32767);
  EXPECT_EQ(nm.layer_get<int16_t>("h", 3), -32768);
  EXPECT_DOUBLE_EQ(nm.layer_get<double>("h", 3), -32768.0);  // read through the double fallback
  EXPECT_EQ(nm.layer_get<uint8_t>("h", 3), 0);               // clamped to the target range
  EXPECT_EQ(nm.layer_get<uint8_t>("h", 4), 255);
  EXPECT_EQ(nm.layer_get<int16_t>("h", 9999, int16_t{7}), 7);  // out of range: default
  // Reading another layer type as int16 keeps negative values (they used to clamp to 0)
  nm.add_layer<float>("f", "", "", -12.6f);
  EXPECT_EQ(nm.layer_get<int16_t>("f", 0), -13);
  nm.layer_set<float>("f", 1, 1e9f);
  EXPECT_EQ(nm.layer_get<int16_t>("f", 1), 32767);
  // And so does creating an int16 layer from a float one
  auto conv = nm.add_layer<int16_t>("f16", "", "", std::string("f"));
  EXPECT_EQ(conv->data()[0], -13);
  EXPECT_EQ(conv->data()[1], 32767);
}

TEST(LayersI16, CopiesAssignmentsAndHashes)
{
  NavMap nm = grid();
  auto v = nm.add_layer<int16_t>("h", "", "", int16_t{1});
  const auto h0 = v->content_hash();
  nm.layer_set<int16_t>("h", 2, -300);
  EXPECT_NE(v->content_hash(), h0);

  NavMap copy(nm);
  EXPECT_EQ(copy.layer_get<int16_t>("h", 2), -300);
  NavMap assigned = grid();
  assigned = nm;
  EXPECT_EQ(assigned.layer_get<int16_t>("h", 2), -300);
  EXPECT_EQ(assigned.layer_type_name("h"), "int16");
  // Re-assigning after a change in the source updates the destination
  nm.layer_set<int16_t>("h", 2, 42);
  assigned = nm;
  EXPECT_EQ(assigned.layer_get<int16_t>("h", 2), 42);
}

// ------------------------------------------------------------------ unknown value

TEST(LayerUnknown, DeclaredPerLayerForIntegersAndNaN)
{
  NavMap nm = grid();
  nm.add_layer<uint8_t>("occ", "", "", uint8_t{0});
  nm.add_layer<float>("elev", "", "m", 0.0f);
  // Without a declared unknown value nothing is unknown
  nm.layer_set<uint8_t>("occ", 1, 255);
  EXPECT_FALSE(nm.is_unknown("occ", 1));
  nm.set_layer_unknown("occ", 255.0);
  EXPECT_TRUE(nm.is_unknown("occ", 1));
  EXPECT_FALSE(nm.is_unknown("occ", 0));
  // NaN as the unknown value of a float layer
  nm.set_layer_unknown("elev", std::numeric_limits<double>::quiet_NaN());
  nm.layer_set<float>("elev", 2, std::numeric_limits<float>::quiet_NaN());
  EXPECT_TRUE(nm.is_unknown("elev", 2));
  EXPECT_FALSE(nm.is_unknown("elev", 3));
  // Missing layer or cell: not unknown (nothing to say)
  EXPECT_FALSE(nm.is_unknown("nope", 0));
  EXPECT_FALSE(nm.is_unknown("elev", 9999));
  EXPECT_FALSE(nm.is_unknown("occ", 9999));
  // Kept by add_layer (which used to reset the metadata) and by copies
  nm.add_layer<uint8_t>("occ", "occupancy", "%", uint8_t{0});
  ASSERT_TRUE(nm.get_layer_meta("occ")->unknown.has_value());
  EXPECT_EQ(*nm.get_layer_meta("occ")->unknown, 255.0);
  EXPECT_EQ(nm.get_layer_meta("occ")->description, "occupancy");
  NavMap copy(nm);
  EXPECT_TRUE(copy.is_unknown("occ", 1));
  EXPECT_TRUE(copy.is_unknown("elev", 2));
}

// ------------------------------------------------------------------ log-odds occupancy

TEST(Occupancy, FixedPointValuesOfTheOctoMapDefaults)
{
  EXPECT_EQ(OccupancyLayer::to_fixed(OccupancyLayer::logodds_of(0.7f)), 847);
  EXPECT_EQ(OccupancyLayer::to_fixed(OccupancyLayer::logodds_of(0.4f)), -405);
  EXPECT_EQ(OccupancyLayer::to_fixed(OccupancyLayer::logodds_of(0.12f)), -1992);
  EXPECT_EQ(OccupancyLayer::to_fixed(OccupancyLayer::logodds_of(0.97f)), 3476);
  EXPECT_EQ(OccupancyLayer::to_fixed(1e9f), 32767);
  EXPECT_EQ(OccupancyLayer::to_fixed(-1e9f), -32767);
  EXPECT_NEAR(OccupancyLayer::probability_of(OccupancyLayer::logodds_of(0.3f)), 0.3f, 1e-6f);
}

TEST(Occupancy, NewCellsAreUnknownAndTheLayerTravelsWithTheMap)
{
  NavMap nm = grid();
  OccupancyLayer occ(nm);
  EXPECT_EQ(nm.layer_type_name("occupancy_logodds"), "int16");
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    EXPECT_EQ(occ.state(c), OccupancyState::UNKNOWN);
    EXPECT_FLOAT_EQ(occ.probability(c), 0.5f);
    EXPECT_TRUE(nm.is_unknown("occupancy_logodds", c));
  }
  occ.hit(3);
  EXPECT_FALSE(nm.is_unknown("occupancy_logodds", 3));
  // A second OccupancyLayer over the same layer sees (and keeps) its values
  OccupancyLayer again(nm);
  EXPECT_EQ(again.state(3), OccupancyState::OCCUPIED);
  NavMap copy(nm);
  EXPECT_EQ(copy.layer_get<int16_t>("occupancy_logodds", 3), 847);
}

TEST(Occupancy, SequencesClampAndClassify)
{
  NavMap nm = grid();
  OccupancyLayer occ(nm);
  // One hit: occupied; one miss: free (exactly at the threshold)
  occ.hit(0);
  EXPECT_EQ(occ.state(0), OccupancyState::OCCUPIED);
  occ.miss(1);
  EXPECT_EQ(occ.state(1), OccupancyState::FREE);
  // Hit then miss: 442 (still occupied); one more miss: 37 (unknown)
  occ.miss(0);
  EXPECT_EQ(nm.layer_get<int16_t>(occ.name(), 0), 442);
  EXPECT_EQ(occ.state(0), OccupancyState::OCCUPIED);
  occ.miss(0);
  EXPECT_EQ(nm.layer_get<int16_t>(occ.name(), 0), 37);
  EXPECT_EQ(occ.state(0), OccupancyState::UNKNOWN);
  // Saturation both ways, and recovery from saturation takes a bounded number of updates
  for (int k = 0; k < 100; ++k) {
    occ.hit(2);
  }
  EXPECT_EQ(nm.layer_get<int16_t>(occ.name(), 2), 3476);
  EXPECT_NEAR(occ.probability(2), 0.97f, 1e-3f);
  int misses = 0;
  while (occ.state(2) != OccupancyState::FREE) {occ.miss(2); ++misses;}
  EXPECT_EQ(misses, (3476 + 405 + 404) / 405);  // from the top to <= -405
  for (int k = 0; k < 100; ++k) {
    occ.miss(4);
  }
  EXPECT_EQ(nm.layer_get<int16_t>(occ.name(), 4), -1992);
  // Reset and out-of-range cells
  occ.reset(4);
  EXPECT_EQ(occ.state(4), OccupancyState::UNKNOWN);
  occ.hit(99999);
  occ.reset(99999);
  EXPECT_EQ(occ.state(99999), OccupancyState::UNKNOWN);
}

TEST(Occupancy, MatchesTheFloatRuleWithinRounding)
{
  // The fixed-point layer must follow OctoMap's float update with clamping.
  NavMap nm = grid();
  OccupancyLayer occ(nm);
  std::mt19937 rng(7);
  std::bernoulli_distribution B(0.6);
  const float lh = OccupancyLayer::logodds_of(0.7f), lm = OccupancyLayer::logodds_of(0.4f);
  const float lo = OccupancyLayer::logodds_of(0.12f), hi = OccupancyLayer::logodds_of(0.97f);
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    float ref = 0.0f;
    for (int k = 0; k < 200; ++k) {
      const bool h = B(rng);
      occ.update(c, h);
      ref = std::clamp(ref + (h ? lh : lm), lo, hi);
      ASSERT_NEAR(occ.logodds(c), ref, 0.0006f * (k + 1) + 1e-3f) << c << " step " << k;
    }
  }
}

TEST(Occupancy, CustomParametersAndDirtyHash)
{
  NavMap nm = grid();
  OccupancyParams p;
  p.p_hit = 0.9f;
  p.p_occupied = 0.95f;
  OccupancyLayer occ(nm, "occ2", p);
  occ.hit(0);
  EXPECT_EQ(occ.state(0), OccupancyState::UNKNOWN);  // 0.9 < 0.95
  occ.hit(0);
  EXPECT_EQ(occ.state(0), OccupancyState::OCCUPIED);
  const auto h = occ.view()->content_hash();
  occ.miss(5);
  EXPECT_NE(occ.view()->content_hash(), h);  // updates invalidate the cached hash
}
