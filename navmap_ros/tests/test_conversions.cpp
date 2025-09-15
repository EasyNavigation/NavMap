// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "navmap_ros/conversions.hpp"
#include "navmap_core/NavMap.hpp"

using navmap_ros::from_occupancy_grid;
using navmap_ros::to_occupancy_grid;

static nav_msgs::msg::OccupancyGrid make_grid_4m_0p1()
{
  nav_msgs::msg::OccupancyGrid g;
  const int W = 40, H = 40;
  g.header.frame_id = "map";
  g.info.width = W;
  g.info.height = H;
  g.info.resolution = 0.1f;
  g.info.origin.position.x = 0.0;
  g.info.origin.position.y = 0.0;
  g.info.origin.position.z = 0.0;
  g.info.origin.orientation.w = 1.0;
  g.data.assign(W * H, 0);

  auto id = [&](int i, int j) { return j * W + i; };

  // Borders 100
  for (int i = 0; i < W; ++i) { g.data[id(i, 0)] = 100; g.data[id(i, H-1)] = 100; }
  for (int j = 0; j < H; ++j) { g.data[id(0, j)] = 100; g.data[id(W-1, j)] = 100; }

  // Central 6x6 block 100 (cells [17..22]x[17..22])
  for (int j = 17; j <= 22; ++j) for (int i = 17; i <= 22; ++i) g.data[id(i, j)] = 100;

  // Cross 50
  for (int k = 10; k < 30; ++k) { g.data[id(20, k)] = 50; g.data[id(k, 20)] = 50; }

  // Unknowns
  g.data[id(5, 5)] = -1;
  g.data[id(34, 17)] = -1;
  return g;
}

static uint8_t occ_to_u8(int8_t v)
{
  if (v < 0) return 255u;
  if (v >= 100) return 254u;
  return static_cast<uint8_t>(std::lround((v / 100.0) * 254.0));
}

static inline navmap::NavCelId tri_index_for_cell(uint32_t i, uint32_t j, uint32_t W)
{
  return static_cast<navmap::NavCelId>((j * W + i) * 2);
}

TEST(TestConversions, RoundTrip_ExactEquality_4m_0p1)
{
  const int W = 40, H = 40;
  auto g = make_grid_4m_0p1();

  auto nm = from_occupancy_grid(g);
  auto gout = to_occupancy_grid(nm);

  ASSERT_EQ(gout.info.width, g.info.width);
  ASSERT_EQ(gout.info.height, g.info.height);
  ASSERT_NEAR(gout.info.resolution, g.info.resolution, 1e-7f);

  EXPECT_NEAR(gout.info.origin.position.x, g.info.origin.position.x, 1e-7);
  EXPECT_NEAR(gout.info.origin.position.y, g.info.origin.position.y, 1e-7);
  EXPECT_NEAR(gout.info.origin.position.z, g.info.origin.position.z, 1e-7);
  EXPECT_NEAR(gout.info.origin.orientation.w, g.info.origin.orientation.w, 1e-7);

  ASSERT_EQ(gout.data.size(), g.data.size());
  for (size_t idx = 0; idx < g.data.size(); ++idx) {
    EXPECT_EQ(gout.data[idx], g.data[idx]) << "Mismatch at cell " << idx;
  }

  // Structural checks (shared vertices)
  ASSERT_EQ(nm.positions.size(), static_cast<size_t>((W + 1) * (H + 1)));
  ASSERT_EQ(nm.navcels.size(), static_cast<size_t>(2 * W * H));
  ASSERT_EQ(nm.surfaces.size(), 1u);

  auto base = nm.layers.get("occupancy");
  ASSERT_TRUE(base && base->type() == navmap::LayerType::U8);
  auto occ = std::dynamic_pointer_cast<navmap::LayerView<uint8_t>>(base);
  ASSERT_EQ(occ->size(), nm.navcels.size());

  // Two triangles per cell must carry identical value equal to source cell.
  for (uint32_t j = 0; j < static_cast<uint32_t>(H); ++j) {
    for (uint32_t i = 0; i < static_cast<uint32_t>(W); ++i) {
      const auto cell = j * W + i;
      const uint8_t exp = occ_to_u8(g.data[cell]);
      const navmap::NavCelId t0 = tri_index_for_cell(i, j, W);
      const navmap::NavCelId t1 = t0 + 1;
      ASSERT_LT(t1, nm.navcels.size());
      EXPECT_EQ((*occ)[t0], exp) << "cell(" << i << "," << j << ")";
      EXPECT_EQ((*occ)[t1], exp) << "cell(" << i << "," << j << ")";
    }
  }
}

TEST(TestConversions, TriangleIndicesFollowPattern0)
{
  const int W = 40;
  auto g = make_grid_4m_0p1();
  auto nm = from_occupancy_grid(g);

  // Pick a cell and verify its triangles reference the expected 4 vertices.
  auto v_id = [W](uint32_t i, uint32_t j) -> navmap::PointId {
    return static_cast<navmap::PointId>(j * (W + 1) + i);
  };

  const uint32_t ci = 10, cj = 11;
  const navmap::NavCelId t0 = tri_index_for_cell(ci, cj, W);
  const navmap::NavCelId t1 = t0 + 1;

  ASSERT_LT(t1, nm.navcels.size());

  const auto & a = nm.navcels[t0];
  const auto & b = nm.navcels[t1];

  EXPECT_EQ(a.v[0], v_id(ci + 0, cj + 0));
  EXPECT_EQ(a.v[1], v_id(ci + 1, cj + 0));
  EXPECT_EQ(a.v[2], v_id(ci + 1, cj + 1));

  EXPECT_EQ(b.v[0], v_id(ci + 0, cj + 0));
  EXPECT_EQ(b.v[1], v_id(ci + 1, cj + 1));
  EXPECT_EQ(b.v[2], v_id(ci + 0, cj + 1));
}
