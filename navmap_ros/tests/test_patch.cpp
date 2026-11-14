// Copyright 2026 Intelligent Robotics Lab
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

#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"

using navmap_ros_interfaces::msg::NavMap;

namespace
{

navmap::NavMap grid_map(int cells = 8)
{
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = "map";
  g.info.resolution = 0.5;
  g.info.width = cells;
  g.info.height = cells;
  g.info.origin.orientation.w = 1.0;
  g.data.assign(cells * cells, 0);
  auto nm = navmap_ros::from_occupancy_grid(g);
  for (std::size_t c = 0; c < nm.navcels.size(); ++c) {
    nm.layer_set<float>("height", static_cast<navmap::NavCelId>(c), 0.1f * c);
  }
  return nm;
}

// What a receiver keeps must be what a full conversion of the map gives.
void expect_same(const NavMap & a, const NavMap & b)
{
  EXPECT_EQ(a.geometry_id, b.geometry_id);
  EXPECT_EQ(a.positions_x, b.positions_x);
  EXPECT_EQ(a.positions_y, b.positions_y);
  EXPECT_EQ(a.positions_z, b.positions_z);
  EXPECT_EQ(a.navcels_v0, b.navcels_v0);
  EXPECT_EQ(a.navcels_v1, b.navcels_v1);
  EXPECT_EQ(a.navcels_v2, b.navcels_v2);
  ASSERT_EQ(a.surfaces.size(), b.surfaces.size());
  for (std::size_t i = 0; i < a.surfaces.size(); ++i) {
    EXPECT_EQ(a.surfaces[i].navcels, b.surfaces[i].navcels) << i;
  }
  ASSERT_EQ(a.layers.size(), b.layers.size());
  for (const auto & la : a.layers) {
    bool found = false;
    for (const auto & lb : b.layers) {
      if (lb.name != la.name) {continue;}
      found = true;
      EXPECT_EQ(la.type, lb.type);
      EXPECT_EQ(la.data_u8, lb.data_u8) << la.name;
      EXPECT_EQ(la.data_f32, lb.data_f32) << la.name;
      EXPECT_EQ(la.data_f64, lb.data_f64) << la.name;
      EXPECT_EQ(la.data_i16, lb.data_i16) << la.name;
    }
    EXPECT_TRUE(found) << la.name;
  }
}

// Rewrites NavCel c onto three new vertices around (x, y), as an in-place edit does.
void rewrite(navmap::NavMap & nm, navmap::NavCelId c, float x, float y)
{
  const auto a = nm.add_vertex(Eigen::Vector3f(x, y, 0.0f));
  const auto b = nm.add_vertex(Eigen::Vector3f(x + 0.3f, y, 0.0f));
  const auto d = nm.add_vertex(Eigen::Vector3f(x, y + 0.3f, 0.0f));
  nm.navcels[c].v[0] = a;
  nm.navcels[c].v[1] = b;
  nm.navcels[c].v[2] = d;
  nm.rebuild_geometry_accels(false, false, false);
}

}  // namespace

TEST(NavMapPatch, ApplyingItGivesTheFullMap)
{
  auto nm = grid_map();
  auto kept = navmap_ros::to_msg(nm);
  const auto base = kept.geometry_id;
  const auto nv = static_cast<std::uint32_t>(nm.positions.x.size());

  rewrite(nm, 5, 10.0f, 10.0f);
  nm.layer_set<std::uint8_t>("occupancy", 5, 200);
  nm.layer_set<float>("height", 5, 7.0f);
  nm.layer_set<std::uint8_t>("occupancy", 9, 33);   // a value only

  const auto p = navmap_ros::to_patch_msg(nm, base, nv, {5, 9}, {0});
  EXPECT_EQ(p.navcel_indices.size(), 2u);
  EXPECT_EQ(p.positions_x.size(), 3u) << "only the new vertices";
  ASSERT_TRUE(navmap_ros::apply_patch(p, kept));
  expect_same(kept, navmap_ros::to_msg(nm));
}

TEST(NavMapPatch, TheMapCanGrow)
{
  auto nm = grid_map();
  auto kept = navmap_ros::to_msg(nm);
  const auto base = kept.geometry_id;
  const auto nv = static_cast<std::uint32_t>(nm.positions.x.size());
  const auto n0 = static_cast<std::uint32_t>(nm.navcels.size());

  // Three NavCels more at the end: one written, two degenerate (as a slack of an in-place map)
  const auto a = nm.add_vertex(Eigen::Vector3f(20.0f, 20.0f, 0.0f));
  const auto b = nm.add_vertex(Eigen::Vector3f(21.0f, 20.0f, 0.0f));
  const auto c = nm.add_vertex(Eigen::Vector3f(20.0f, 21.0f, 0.0f));
  nm.add_navcel(a, b, c);
  nm.add_navcel(0, 0, 0);
  nm.add_navcel(0, 0, 0);
  nm.rebuild_geometry_accels(false, false, false);
  nm.layers.resize_all(nm.navcels.size());
  nm.layer_set<std::uint8_t>("occupancy", n0, 42);

  const auto p = navmap_ros::to_patch_msg(nm, base, nv, {n0, n0 + 1, n0 + 2}, {});
  ASSERT_TRUE(navmap_ros::apply_patch(p, kept));
  expect_same(kept, navmap_ros::to_msg(nm));
}

TEST(NavMapPatch, PatchesInAChain)
{
  auto nm = grid_map();
  auto kept = navmap_ros::to_msg(nm);
  for (int k = 0; k < 4; ++k) {
    const auto base = nm.geometry_id();
    const auto nv = static_cast<std::uint32_t>(nm.positions.x.size());
    const auto c = static_cast<navmap::NavCelId>(10 + k);
    rewrite(nm, c, 30.0f + k, 30.0f);
    nm.layer_set<float>("height", c, -1.0f * k);
    ASSERT_TRUE(navmap_ros::apply_patch(navmap_ros::to_patch_msg(nm, base, nv, {c}, {}), kept));
  }
  expect_same(kept, navmap_ros::to_msg(nm));
}

TEST(NavMapPatch, AnotherBaseIsRejectedAndNothingChanges)
{
  auto nm = grid_map();
  auto kept = navmap_ros::to_msg(nm);
  const auto before = kept;
  const auto nv = static_cast<std::uint32_t>(nm.positions.x.size());
  rewrite(nm, 3, 5.0f, 5.0f);
  auto p = navmap_ros::to_patch_msg(nm, before.geometry_id + 1, nv, {3}, {});
  EXPECT_FALSE(navmap_ros::apply_patch(p, kept));
  EXPECT_EQ(kept, before);
}

TEST(NavMapPatch, AMalformedPatchIsRejectedAndNothingChanges)
{
  auto nm = grid_map();
  auto kept = navmap_ros::to_msg(nm);
  const auto before = kept;
  const auto nv = static_cast<std::uint32_t>(nm.positions.x.size());
  rewrite(nm, 3, 5.0f, 5.0f);
  const auto good = navmap_ros::to_patch_msg(nm, before.geometry_id, nv, {3}, {});

  auto p = good;
  p.navcel_indices[0] = p.navcels_size;     // out of range
  EXPECT_FALSE(navmap_ros::apply_patch(p, kept));
  p = good;
  p.navcels_v0[0] = 1000000;                // a vertex nobody has
  EXPECT_FALSE(navmap_ros::apply_patch(p, kept));
  p = good;
  p.first_vertex = nv + 5;                  // vertices missing in between
  EXPECT_FALSE(navmap_ros::apply_patch(p, kept));
  p = good;
  p.layers[0].data_u8.clear();              // values missing
  p.layers[0].data_f32.clear();
  EXPECT_FALSE(navmap_ros::apply_patch(p, kept));
  EXPECT_EQ(kept, before);
}

TEST(NavMapPatch, ANewLayerIsCreated)
{
  auto nm = grid_map();
  auto kept = navmap_ros::to_msg(nm);
  const auto base = kept.geometry_id;
  const auto nv = static_cast<std::uint32_t>(nm.positions.x.size());
  nm.add_layer<std::uint8_t>("costs", "", "", 0);
  nm.layer_set<std::uint8_t>("costs", 4, 99);
  // Same geometry: only a layer value; every NavCel listed so the new layer is complete
  std::vector<std::uint32_t> all(nm.navcels.size());
  for (std::size_t c = 0; c < all.size(); ++c) {
    all[c] = static_cast<std::uint32_t>(c);
  }
  ASSERT_TRUE(navmap_ros::apply_patch(navmap_ros::to_patch_msg(nm, base, nv, all, {}), kept));
  expect_same(kept, navmap_ros::to_msg(nm));
}

TEST(NavMapPatch, SurfacesAreReplaced)
{
  auto nm = grid_map();
  auto kept = navmap_ros::to_msg(nm);
  const auto base = kept.geometry_id;
  const auto nv = static_cast<std::uint32_t>(nm.positions.x.size());
  nm.surfaces[0].navcels.resize(10);
  nm.create_surface("map");
  nm.surfaces[1].navcels = {11, 12};
  ASSERT_TRUE(navmap_ros::apply_patch(navmap_ros::to_patch_msg(nm, base, nv, {}, {0, 1}), kept));
  ASSERT_EQ(kept.surfaces.size(), 2u);
  EXPECT_EQ(kept.surfaces[0].navcels.size(), 10u);
  EXPECT_EQ(kept.surfaces[1].navcels, (std::vector<std::uint32_t>{11, 12}));
}
