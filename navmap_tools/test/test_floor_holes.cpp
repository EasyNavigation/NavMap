// Copyright 2026 José Miguel Guerrero
//
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

#include <array>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>

#include "navmap_tools/floor_holes.hpp"

namespace
{
// A grid of n x n cells of 0.4 m at z = 0 with the cells of `hole` left out
::navmap::NavMap grid_floor(int n, const std::function<bool(int, int)> & hole)
{
  ::navmap::NavMap nm;
  nm.surfaces.resize(1);
  for (int j = 0; j <= n; ++j) {
    for (int i = 0; i <= n; ++i) {
      nm.positions.x.push_back(0.4f * static_cast<float>(i));
      nm.positions.y.push_back(0.4f * static_cast<float>(j));
      nm.positions.z.push_back(0.0f);
    }
  }
  const auto id = [n](int a, int b) {return static_cast<std::uint32_t>(b * (n + 1) + a);};
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      if (hole(i, j)) {continue;}
      for (const auto & t : {std::array<std::uint32_t, 3>{id(i, j), id(i + 1, j), id(i + 1, j + 1)},
          std::array<std::uint32_t, 3>{id(i, j), id(i + 1, j + 1), id(i, j + 1)}})
      {
        ::navmap::NavCel c;
        c.v[0] = t[0]; c.v[1] = t[1]; c.v[2] = t[2];
        nm.surfaces[0].navcels.push_back(static_cast<::navmap::NavCelId>(nm.navcels.size()));
        nm.navcels.push_back(c);
      }
    }
  }
  nm.rebuild_geometry_accels();
  return nm;
}
}  // namespace

TEST(FloorHoles, SmallHolesInTheFloorAreClosedAndLargeOnesAreNot)
{
  // a floor of 30 x 30 cells with a hole of 3 x 3 cells (1.2 m) and one of 10 x 10 (4 m)
  const auto holes = [](int i, int j) {
      return (i >= 5 && i < 8 && j >= 5 && j < 8) || (i >= 15 && i < 25 && j >= 15 && j < 25);
    };
  const auto cells = navmap_tools::floor_hole_cells(grid_floor(30, holes), 0.4f, 2.4f);
  // the small hole is gone (9 cells), the large one stays (its middle at least)
  EXPECT_GE(cells.size(), 9u);
  EXPECT_LT(cells.size(), 9u + 100u - 36u + 1u);
}

TEST(FloorHoles, AStraightBorderDoesNotGrow)
{
  const auto cells = navmap_tools::floor_hole_cells(
    grid_floor(20, [](int, int) {return false;}), 0.4f, 2.4f);
  EXPECT_TRUE(cells.empty());
}

TEST(FloorHoles, ZeroWidthLeavesTheHoles)
{
  const auto cells = navmap_tools::floor_hole_cells(
    grid_floor(20, [](int i, int j) {return i == 10 && j == 10;}), 0.4f, 0.0f);
  EXPECT_TRUE(cells.empty());
}

TEST(FloorHoles, AMapTooFarFromTheOriginSaysToRecentreIt)
{
  // UTM: x = 440 km at cells of 0.25 m is past the 2^20 cells the index holds; before, the
  // indices collided and floor was added at wrong places
  auto nm = grid_floor(4, [](int, int) {return false;});
  for (auto & x : nm.positions.x) {
    x += 440000.0f;
  }
  nm.rebuild_geometry_accels();
  try {
    navmap_tools::floor_hole_cells(nm, 0.25f, 2.0f);
    FAIL() << "no error";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("--recenter"), std::string::npos) << e.what();
  }
}
