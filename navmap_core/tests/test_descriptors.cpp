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

#include "navmap_core/Descriptors.hpp"
#include "navmap_core/NavMap.hpp"

using navmap::DescriptorParams;
using navmap::NavCelId;
using navmap::NavMap;

namespace
{

void add_grid(
  NavMap & nm, int nx, int ny, float res, const std::function<float(float, float)> & f)
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
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j), v(i + 1, j + 1)));
      nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j + 1), v(i, j + 1)));
    }
  }
}

float at(const NavMap & nm, const std::string & layer, float x, float y, float z)
{
  std::size_t s; NavCelId c; Eigen::Vector3f bary, hit;
  EXPECT_TRUE(nm.locate_navcel({x, y, z}, s, c, bary, &hit));
  return nm.layer_get<float>(layer, c);
}

// Interior NavCels (away from the map border by `margin`)
std::vector<NavCelId> interior(const NavMap & nm, float x0, float x1, float y0, float y1)
{
  std::vector<NavCelId> out;
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    const auto p = nm.navcel_centroid(c);
    if (p.x() > x0 && p.x() < x1 && p.y() > y0 && p.y() < y1) {out.push_back(c);}
  }
  return out;
}

}  // namespace

TEST(Descriptors, FlatGroundIsAllZeroWithNothingAbove)
{
  NavMap nm;
  add_grid(nm, 30, 30, 0.1f, [](float, float) {return 0.0f;});
  nm.rebuild_geometry_accels();
  navmap::compute_descriptors(nm);
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    EXPECT_NEAR(nm.layer_get<float>("slope", c), 0.0f, 1e-3f);
    EXPECT_NEAR(nm.layer_get<float>("roughness", c), 0.0f, 1e-5f);
    EXPECT_NEAR(nm.layer_get<float>("step", c), 0.0f, 1e-6f);
    EXPECT_TRUE(std::isinf(nm.layer_get<float>("clearance", c)));
  }
  EXPECT_EQ(nm.get_layer_meta("slope")->unit, "deg");
  EXPECT_EQ(nm.get_layer_meta("clearance")->unit, "m");
}

TEST(Descriptors, AnInclineHasItsSlopeAStepOfItsDiscAndNoRoughness)
{
  const float th = 20.0f * static_cast<float>(M_PI) / 180.0f;
  NavMap nm;
  add_grid(nm, 40, 20, 0.1f, [th](float x, float) {return std::tan(th) * x;});
  nm.rebuild_geometry_accels();
  navmap::compute_descriptors(nm);
  for (NavCelId c : interior(nm, 0.5f, 3.5f, 0.5f, 1.5f)) {
    EXPECT_NEAR(nm.layer_get<float>("slope", c), 20.0f, 0.01f);
    EXPECT_LT(nm.layer_get<float>("roughness", c), 1e-4f);
    // Vertices of the NavCels within 0.2 m: about 0.4-0.6 m across along the incline
    const float step = nm.layer_get<float>("step", c);
    EXPECT_GE(step, 0.39f * std::tan(th));
    EXPECT_LE(step, 0.65f * std::tan(th));
  }
}

TEST(Descriptors, RoughnessFollowsTheNoise)
{
  for (const float sigma : {0.005f, 0.02f, 0.05f}) {
    NavMap nm;
    std::mt19937 rng(3);
    std::normal_distribution<float> N(0.0f, sigma);
    add_grid(nm, 40, 40, 0.1f, [&](float, float) {return N(rng);});
    nm.rebuild_geometry_accels();
    DescriptorParams p;
    p.roughness_radius = 0.4f;
    navmap::compute_descriptors(nm, p);
    std::vector<float> r;
    for (NavCelId c : interior(nm, 0.5f, 3.5f, 0.5f, 3.5f)) {
      r.push_back(nm.layer_get<float>("roughness", c));
    }
    std::nth_element(r.begin(), r.begin() + r.size() / 2, r.end());
    const float med = r[r.size() / 2];
    // Centroids average three vertices: their spread is about sigma / sqrt(3)
    EXPECT_GT(med, 0.4f * sigma) << sigma;
    EXPECT_LT(med, 0.8f * sigma) << sigma;
  }
}

TEST(Descriptors, ACurbIsAStepAndASteepStrip)
{
  // 0.15 m curb rising over one 0.1 m column at x in [2.0, 2.1]
  NavMap nm;
  add_grid(nm, 40, 20, 0.1f, [](float x, float) {return x > 2.05f ? 0.15f : 0.0f;});
  nm.rebuild_geometry_accels();
  navmap::compute_descriptors(nm);
  EXPECT_NEAR(at(nm, "step", 1.95f, 1.0f, 0.0f), 0.15f, 1e-5f);   // next to it
  EXPECT_NEAR(at(nm, "step", 2.15f, 1.0f, 0.15f), 0.15f, 1e-5f);
  EXPECT_NEAR(at(nm, "step", 0.8f, 1.0f, 0.0f), 0.0f, 1e-6f);     // far from it
  EXPECT_NEAR(at(nm, "step", 3.5f, 1.0f, 0.15f), 0.0f, 1e-6f);
  EXPECT_GT(at(nm, "slope", 2.05f, 1.0f, 0.07f), 50.0f);          // the riser
  EXPECT_LT(at(nm, "slope", 0.8f, 1.0f, 0.0f), 0.01f);
}

TEST(Descriptors, StackedFloorsClearanceAndNoMixing)
{
  NavMap nm;
  add_grid(nm, 20, 20, 0.1f, [](float, float) {return 0.0f;});
  add_grid(nm, 20, 20, 0.1f, [](float, float) {return 3.0f;});
  nm.rebuild_geometry_accels();
  navmap::compute_descriptors(nm);
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    const bool lower = nm.navcel_centroid(c).z() < 1.0f;
    const float cl = nm.layer_get<float>("clearance", c);
    if (lower) {EXPECT_NEAR(cl, 3.0f, 1e-3f);} else {EXPECT_TRUE(std::isinf(cl));}
    // Discs stay on their floor: no 3 m step, no roughness from the other floor
    EXPECT_NEAR(nm.layer_get<float>("step", c), 0.0f, 1e-6f);
    EXPECT_NEAR(nm.layer_get<float>("roughness", c), 0.0f, 1e-5f);
  }
  // Beyond clearance_max there is nothing above
  DescriptorParams p;
  p.clearance_max = 2.0f;
  navmap::compute_descriptors(nm, p);
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    EXPECT_TRUE(std::isinf(nm.layer_get<float>("clearance", c)));
  }
}

TEST(Descriptors, OnlyTheSelectedOnesAndExistingLayersAreOverwritten)
{
  NavMap nm;
  add_grid(nm, 10, 10, 0.1f, [](float, float) {return 0.0f;});
  nm.rebuild_geometry_accels();
  nm.add_layer<float>("slope", "", "", 99.0f);
  DescriptorParams p;
  p.roughness = p.step = p.clearance = false;
  navmap::compute_descriptors(nm, p);
  EXPECT_TRUE(nm.has_layer("slope"));
  EXPECT_FALSE(nm.has_layer("roughness"));
  EXPECT_FALSE(nm.has_layer("step"));
  EXPECT_FALSE(nm.has_layer("clearance"));
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    EXPECT_NEAR(nm.layer_get<float>("slope", c), 0.0f, 1e-3f);
  }
  EXPECT_TRUE(nm.layers.get("slope")->all_changed());
  // An empty map: nothing to do, no crash
  NavMap empty;
  navmap::compute_descriptors(empty);
  EXPECT_EQ(empty.layer_size("slope"), 0u);
}
