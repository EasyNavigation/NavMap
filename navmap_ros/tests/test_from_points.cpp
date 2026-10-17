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
#include <map>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"
#include "pcl_conversions/pcl_conversions.h"

using navmap::NavCelId;
using navmap::NavMap;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

namespace
{

constexpr float kRes = 0.5f;

navmap_ros::BuildParams params(float res = kRes)
{
  navmap_ros::BuildParams p;
  p.resolution = res;
  p.max_dz = 0.25f;
  p.max_slope_deg = 30.0f;
  return p;
}

// Dense samples (0.1 m) of z = f(x, y) over [x0, x1] x [y0, y1].
void sample(
  Cloud & cloud, float x0, float x1, float y0, float y1,
  const std::function<float(float, float)> & f, float noise = 0.0f, unsigned seed = 1)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> N(-noise, noise);
  for (float x = x0 + 0.05f; x < x1; x += 0.1f) {
    for (float y = y0 + 0.05f; y < y1; y += 0.1f) {
      cloud.push_back(pcl::PointXYZ(x, y, f(x, y) + (noise > 0.0f ? N(rng) : 0.0f)));
    }
  }
}

NavMap build(const Cloud & cloud, navmap_ros::BuildParams p = params())
{
  navmap_ros_interfaces::msg::NavMap msg;
  return navmap_ros::from_points(cloud, msg, p);
}

float total_area(const NavMap & nm)
{
  float a = 0.0f;
  for (const auto & c : nm.navcels) {
    a += c.area;
  }
  return a;
}

// Max number of cells of one surface stacked at the same height over any sample point.
int max_same_height_cover(const NavMap & nm, int samples = 1500, unsigned seed = 3)
{
  Eigen::Vector3f lo(1e9f, 1e9f, 1e9f), hi(-1e9f, -1e9f, -1e9f);
  for (std::size_t v = 0; v < nm.positions.size(); ++v) {
    lo = lo.cwiseMin(nm.positions.at(static_cast<uint32_t>(v)));
    hi = hi.cwiseMax(nm.positions.at(static_cast<uint32_t>(v)));
  }
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> X(lo.x(), hi.x()), Y(lo.y(), hi.y());
  int worst = 0;
  for (int s = 0; s < samples; ++s) {
    const Eigen::Vector3f o(X(rng), Y(rng), hi.z() + 10.0f), d(0.0f, 0.0f, -1.0f);
    std::map<std::size_t, std::vector<float>> hits;  // surface -> hit heights
    for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
      const auto & c = nm.navcels[cid];
      float t, u, v;
      if (navmap::ray_triangle_intersect(
          o, d, nm.positions.at(c.v[0]), nm.positions.at(c.v[1]), nm.positions.at(c.v[2]),
          t, u, v) && u > 1e-3f && v > 1e-3f && u + v < 1.0f - 1e-3f)
      {
        hits[nm.surface_of(cid)].push_back(o.z() - t);
      }
    }
    for (auto & [s_idx, zs] : hits) {
      std::sort(zs.begin(), zs.end());
      int run = 1;
      for (std::size_t k = 1; k < zs.size(); ++k) {
        run = (zs[k] - zs[k - 1] < 0.2f) ? run + 1 : 1;
        worst = std::max(worst, run);
      }
      worst = std::max(worst, 1);
    }
  }
  return worst;
}

// Every edge used by at most two cells and every vertex referenced.
void expect_clean_mesh(const NavMap & nm)
{
  EXPECT_EQ(nm.nonmanifold_edges(), 0u);
  std::map<std::pair<uint32_t, uint32_t>, int> edge_uses;
  std::vector<char> used(nm.positions.size(), 0);
  for (const auto & c : nm.navcels) {
    for (int e = 0; e < 3; ++e) {
      const auto a = c.v[e], b = c.v[(e + 1) % 3];
      ++edge_uses[{std::min(a, b), std::max(a, b)}];
      used[a] = 1;
    }
    EXPECT_GT(c.normal.z(), 0.0f);  // upward facing
  }
  for (const auto & [edge, n] : edge_uses) {
    EXPECT_LE(n, 2);
  }
  EXPECT_EQ(std::count(used.begin(), used.end(), 0), 0) << "unused vertices";
  for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
    EXPECT_NE(nm.surface_of(cid), NavMap::kNoSurface);
  }
  ASSERT_TRUE(nm.has_layer("elevation"));
  EXPECT_EQ(nm.layer_size("elevation"), nm.navcels.size());
}

bool connected(const NavMap & nm, NavCelId from, NavCelId to)
{
  std::vector<char> seen(nm.navcels.size(), 0);
  std::vector<NavCelId> stack{from};
  seen[from] = 1;
  while (!stack.empty()) {
    const NavCelId c = stack.back();
    stack.pop_back();
    if (c == to) {return true;}
    for (NavCelId n : nm.navcel_neighbors(c)) {
      if (!seen[n]) {seen[n] = 1; stack.push_back(n);}
    }
  }
  return false;
}

NavCelId cell_at(const NavMap & nm, float x, float y, float z, float * hit_z = nullptr)
{
  std::size_t s; NavCelId cid; Eigen::Vector3f bary, hit;
  EXPECT_TRUE(nm.locate_navcel({x, y, z}, s, cid, bary, &hit)) << x << "," << y << "," << z;
  if (hit_z) {*hit_z = hit.z();}
  return cid;
}

auto flat(float h) {return [h](float, float) {return h;};}

}  // namespace

TEST(FromPoints, FlatPlaneIsAFullManifoldGrid)
{
  Cloud cloud;
  sample(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f));
  const NavMap nm = build(cloud);

  // Samples span 9.9 m: 21 x 21 cells (half-cell origin shift) -> 20 x 20 quads.
  EXPECT_EQ(nm.navcels.size(), 2u * 20u * 20u);
  EXPECT_EQ(nm.positions.size(), 21u * 21u);
  EXPECT_NEAR(total_area(nm), 10.0f * 10.0f, 1e-2f);
  EXPECT_EQ(nm.surfaces.size(), 1u);
  expect_clean_mesh(nm);
  EXPECT_EQ(max_same_height_cover(nm), 1);
}

TEST(FromPoints, NoisyPlaneStaysOneSheet)
{
  Cloud cloud;
  sample(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f), 0.04f);
  const NavMap nm = build(cloud);
  EXPECT_EQ(nm.navcels.size(), 2u * 20u * 20u);
  EXPECT_EQ(nm.surfaces.size(), 1u);
  expect_clean_mesh(nm);
  EXPECT_EQ(max_same_height_cover(nm), 1);
}

TEST(FromPoints, TwoStackedFloorsAreTwoSurfaces)
{
  Cloud cloud;
  sample(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f));
  sample(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(3.0f));
  const NavMap nm = build(cloud);

  EXPECT_EQ(nm.navcels.size(), 2u * 2u * 20u * 20u);
  ASSERT_EQ(nm.surfaces.size(), 2u);
  expect_clean_mesh(nm);
  EXPECT_EQ(max_same_height_cover(nm), 1);

  float z_low, z_high;
  const NavCelId low = cell_at(nm, 5.2f, 5.3f, 0.05f, &z_low);
  const NavCelId high = cell_at(nm, 5.2f, 5.3f, 3.05f, &z_high);
  EXPECT_NEAR(z_low, 0.0f, 1e-4f);
  EXPECT_NEAR(z_high, 3.0f, 1e-4f);
  EXPECT_NE(nm.surface_of(low), nm.surface_of(high));
  EXPECT_FALSE(connected(nm, low, high));
}

TEST(FromPoints, RampConnectsGroundAndUpperFloor)
{
  // Ground y in [0, 12]; over y in [0, 6] a 15 deg ramp (x in [10, 20]) leads to an upper
  // floor over the ground (x > 20). The ground under the upper floor is reached from the side.
  const float slope = std::tan(15.0f * static_cast<float>(M_PI) / 180.0f);
  const float h = slope * 10.0f;
  Cloud cloud;
  sample(cloud, 0.0f, 30.0f, 0.0f, 12.0f, flat(0.0f));
  sample(cloud, 10.0f, 20.0f, 0.0f, 6.0f, [&](float x, float) {return slope * (x - 10.0f);});
  sample(cloud, 20.0f, 30.0f, 0.0f, 6.0f, flat(h));
  const NavMap nm = build(cloud);

  expect_clean_mesh(nm);
  EXPECT_EQ(max_same_height_cover(nm), 1);

  float z;
  const NavCelId ground = cell_at(nm, 5.0f, 3.0f, 0.05f, &z);
  EXPECT_NEAR(z, 0.0f, 1e-3f);
  const NavCelId upper = cell_at(nm, 25.0f, 3.0f, h + 0.05f, &z);
  EXPECT_NEAR(z, h, 1e-3f);
  const NavCelId below_upper = cell_at(nm, 25.0f, 3.0f, 0.05f, &z);
  EXPECT_NEAR(z, 0.0f, 1e-3f);
  const NavCelId on_ramp = cell_at(nm, 15.2f, 3.0f, slope * 5.2f + 0.05f, &z);
  EXPECT_NEAR(z, slope * 5.2f, 0.15f);
  const NavCelId under_ramp = cell_at(nm, 17.0f, 3.0f, 0.05f, &z);
  EXPECT_NEAR(z, 0.0f, 1e-3f);

  EXPECT_TRUE(connected(nm, ground, upper));
  EXPECT_TRUE(connected(nm, on_ramp, upper));
  EXPECT_TRUE(connected(nm, ground, below_upper));  // around, by y > 6
  EXPECT_NE(upper, below_upper);
  EXPECT_NE(on_ramp, under_ramp);
  // No cell joins the upper floor straight down to the ground below it.
  for (NavCelId n : nm.navcel_neighbors(upper)) {
    EXPECT_GT(nm.navcel_centroid(n).z(), h - 0.5f);
  }
}

TEST(FromPoints, HighStepSplitsLowStepJoins)
{
  for (const auto & [step, expected] : {std::pair{0.6f, 2u}, std::pair{0.1f, 1u}}) {
    Cloud cloud;
    sample(cloud, 0.0f, 10.0f, 0.0f, 5.0f, flat(0.0f));
    sample(cloud, 10.0f, 20.0f, 0.0f, 5.0f, flat(step));
    const NavMap nm = build(cloud);
    EXPECT_EQ(nm.surfaces.size(), expected) << "step " << step;
    expect_clean_mesh(nm);
  }
}

TEST(FromPoints, SlopeLimit)
{
  for (const auto & [deg, meshed] : {std::pair{20.0f, true}, std::pair{45.0f, false}}) {
    const float s = std::tan(deg * static_cast<float>(M_PI) / 180.0f);
    Cloud cloud;
    sample(cloud, 0.0f, 10.0f, 0.0f, 5.0f, [s](float x, float) {return s * x;});
    const NavMap nm = build(cloud);
    if (meshed) {
      EXPECT_EQ(nm.navcels.size(), 2u * 20u * 10u) << deg;
      expect_clean_mesh(nm);
    } else {
      // Nothing inside the steep slope; a partial border cell may leave a rim strip.
      for (const auto & c : nm.navcels) {
        EXPECT_LE(std::acos(c.normal.z()), 30.0f * static_cast<float>(M_PI) / 180.0f + 1e-4f);
      }
      for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
        const float x = nm.navcel_centroid(cid).x();
        EXPECT_TRUE(x<0.6f || x>9.4f) << deg << " centroid x " << x;
      }
    }
  }
}

TEST(FromPoints, IsolatedOutliersAddNothing)
{
  Cloud cloud;
  sample(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f));
  std::mt19937 rng(9);
  std::uniform_real_distribution<float> U(0.0f, 10.0f);
  for (int k = 0; k < 15; ++k) {
    cloud.push_back(pcl::PointXYZ(U(rng), U(rng), 1.5f + 0.1f * k));
  }
  const NavMap nm = build(cloud);
  EXPECT_EQ(nm.navcels.size(), 2u * 20u * 20u);
  EXPECT_EQ(nm.surfaces.size(), 1u);
  expect_clean_mesh(nm);
}

TEST(FromPoints, WallSplitsTheFloor)
{
  Cloud cloud;
  sample(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f));
  for (float y = 0.05f; y < 10.0f; y += 0.1f) {
    for (float z = 0.0f; z < 2.0f; z += 0.05f) {
      cloud.push_back(pcl::PointXYZ(5.1f, y, z));
    }
  }
  const NavMap nm = build(cloud);
  expect_clean_mesh(nm);
  const NavCelId left = cell_at(nm, 2.0f, 5.0f, 0.05f);
  const NavCelId right = cell_at(nm, 8.0f, 5.0f, 0.05f);
  EXPECT_FALSE(connected(nm, left, right));
}

TEST(FromPoints, MaxSurfacesDropsTrianglesAndVertices)
{
  Cloud cloud;
  sample(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f));
  sample(cloud, 0.0f, 4.0f, 0.0f, 4.0f, flat(3.0f));  // smaller upper floor
  auto p = params();
  p.max_surfaces = 1;
  navmap_ros_interfaces::msg::NavMap msg;
  const NavMap nm = navmap_ros::from_points(cloud, msg, p);

  ASSERT_EQ(nm.surfaces.size(), 1u);
  EXPECT_EQ(nm.navcels.size(), 2u * 20u * 20u);  // the larger, lower floor
  EXPECT_EQ(nm.surfaces[0].navcels.size(), nm.navcels.size());
  expect_clean_mesh(nm);
  for (std::size_t v = 0; v < nm.positions.size(); ++v) {
    EXPECT_NEAR(nm.positions.z[v], 0.0f, 1e-5f);
  }
  EXPECT_EQ(msg.navcels_v0.size(), nm.navcels.size());
  EXPECT_EQ(msg.layers[0].data_f32.size(), nm.navcels.size());
}

TEST(FromPoints, ElevationLayerIsMeanVertexHeight)
{
  Cloud cloud;
  sample(cloud, 0.0f, 10.0f, 0.0f, 5.0f, [](float x, float y) {return 0.1f * x + 0.05f * y;});
  const NavMap nm = build(cloud);
  ASSERT_FALSE(nm.navcels.empty());
  for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
    const auto & c = nm.navcels[cid];
    const float mean = (nm.positions.z[c.v[0]] + nm.positions.z[c.v[1]] +
      nm.positions.z[c.v[2]]) / 3.0f;
    EXPECT_NEAR(nm.layer_get<float>("elevation", cid), mean, 1e-5f);
  }
}

TEST(FromPoints, PointOrderDoesNotMatter)
{
  Cloud cloud;
  sample(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f), 0.03f);
  sample(cloud, 2.0f, 8.0f, 2.0f, 8.0f, flat(2.8f), 0.03f, 2);
  const NavMap a = build(cloud);
  std::mt19937 rng(4);
  std::shuffle(cloud.points.begin(), cloud.points.end(), rng);
  const NavMap b = build(cloud);

  ASSERT_EQ(a.navcels.size(), b.navcels.size());
  EXPECT_EQ(a.surfaces.size(), b.surfaces.size());
  EXPECT_NEAR(total_area(a), total_area(b), 1e-3f);
  auto verts = [](const NavMap & nm) {
      std::set<std::tuple<float, float, float>> s;
      for (std::size_t v = 0; v < nm.positions.size(); ++v) {
        s.emplace(nm.positions.x[v], nm.positions.y[v], nm.positions.z[v]);
      }
      return s;
    };
  EXPECT_EQ(verts(a), verts(b));
}

TEST(FromPoints, DegenerateInputsGiveEmptyMaps)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  Cloud empty, two, nans, flat_ok;
  two.push_back(pcl::PointXYZ(0, 0, 0));
  two.push_back(pcl::PointXYZ(1, 0, 0));
  for (int k = 0; k < 10; ++k) {
    nans.push_back(pcl::PointXYZ(nan, k, 0));
                                                                         }
  sample(flat_ok, 0.0f, 5.0f, 0.0f, 5.0f, flat(0.0f));

  EXPECT_TRUE(build(empty).navcels.empty());
  EXPECT_TRUE(build(two).navcels.empty());
  EXPECT_TRUE(build(nans).navcels.empty());
  EXPECT_TRUE(build(flat_ok, params(0.0f)).navcels.empty());
  EXPECT_TRUE(build(flat_ok, params(-1.0f)).navcels.empty());

  // A single row of cells has no quads.
  Cloud row;
  sample(row, 0.0f, 10.0f, 0.0f, 0.2f, flat(0.0f));
  EXPECT_TRUE(build(row).navcels.empty());
}

TEST(FromPoints, NonFinitePointsAreIgnored)
{
  Cloud cloud;
  sample(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f));
  const float inf = std::numeric_limits<float>::infinity();
  cloud.push_back(pcl::PointXYZ(inf, 1.0f, 0.0f));
  cloud.push_back(pcl::PointXYZ(1.0f, std::numeric_limits<float>::quiet_NaN(), 0.0f));
  const NavMap nm = build(cloud);
  EXPECT_EQ(nm.navcels.size(), 2u * 20u * 20u);
  expect_clean_mesh(nm);
}

TEST(FromPoints, PointCloud2EntryMatchesPcl)
{
  Cloud cloud;
  sample(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f));
  sample(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(3.0f));
  sensor_msgs::msg::PointCloud2 pc2;
  pcl::toROSMsg(cloud, pc2);
  navmap_ros_interfaces::msg::NavMap msg;
  const NavMap from_pc2 = navmap_ros::from_pointcloud2(pc2, msg, params());
  const NavMap from_pcl = build(cloud);
  EXPECT_EQ(from_pc2.navcels.size(), from_pcl.navcels.size());
  EXPECT_EQ(from_pc2.surfaces.size(), from_pcl.surfaces.size());
}

TEST(FromPoints, MinClearanceDecidesWhatIsALevel)
{
  // Table top 0.7 m over part of the floor.
  Cloud cloud;
  sample(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f));
  sample(cloud, 3.0f, 6.0f, 3.0f, 6.0f, flat(0.7f));

  // Default clearance (1 m): table and floor are one column, the floor under it is lost.
  const NavMap merged = build(cloud);
  expect_clean_mesh(merged);
  float z;
  cell_at(merged, 4.5f, 4.5f, 0.05f, &z);
  EXPECT_GT(z, 0.5f);  // only the table top remains there

  // Low robot (0.5 m): the floor under the table is a level of its own.
  auto p = params();
  p.min_clearance = 0.5f;
  const NavMap split = build(cloud, p);
  expect_clean_mesh(split);
  cell_at(split, 4.5f, 4.5f, 0.05f, &z);
  EXPECT_NEAR(z, 0.0f, 1e-4f);
  cell_at(split, 4.5f, 4.5f, 0.75f, &z);
  EXPECT_NEAR(z, 0.7f, 1e-4f);
}

TEST(FromPoints, VoxelizedCloudGivesOneSheet)
{
  // Gentle slope sampled as voxel centres: x, y, z all quantized to 0.3 m.
  Cloud cloud;
  for (int i = 0; i < 60; ++i) {
    for (int j = 0; j < 30; ++j) {
      const float x = 0.3f * i, y = 0.3f * j;
      const float z = 0.3f * std::round((0.1f * x) / 0.3f);
      cloud.push_back(pcl::PointXYZ(x, y, z));
      cloud.push_back(pcl::PointXYZ(x, y, z - 0.3f));  // surface voxels two deep
    }
  }
  const NavMap nm = build(cloud, params(0.6f));
  expect_clean_mesh(nm);
  EXPECT_EQ(nm.surfaces.size(), 1u);
  // Covers the 17.7 x 8.7 m footprint, up to one cell of border.
  EXPECT_GT(total_area(nm), 17.7f * 8.7f * 0.95f);
  EXPECT_LT(total_area(nm), 18.3f * 9.3f * 1.05f);
}

TEST(FromPoints, LatticeAlignedSamplesFillEveryCell)
{
  // One point per cell exactly on a res-spaced lattice (worst case for binning).
  Cloud cloud;
  for (int i = 0; i < 40; ++i) {
    for (int j = 0; j < 40; ++j) {
      cloud.push_back(pcl::PointXYZ(0.3f * i, 0.3f * j, 0.0f));
    }
  }
  const NavMap nm = build(cloud, params(0.3f));
  EXPECT_EQ(nm.positions.size(), 40u * 40u);
  EXPECT_EQ(nm.navcels.size(), 2u * 39u * 39u);
  expect_clean_mesh(nm);
}

TEST(FromPoints, MinSurfaceAreaDropsSmallSurfaces)
{
  Cloud cloud;
  sample(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f));
  sample(cloud, 0.0f, 2.0f, 0.0f, 2.0f, flat(3.0f));  // ~2.25 m2 upper patch
  auto p = params();
  p.min_surface_area = 5.0f;
  const NavMap nm = build(cloud, p);
  ASSERT_EQ(nm.surfaces.size(), 1u);
  EXPECT_EQ(nm.navcels.size(), 2u * 20u * 20u);
  expect_clean_mesh(nm);

  p.min_surface_area = 1000.0f;  // nothing survives
  EXPECT_TRUE(build(cloud, p).navcels.empty());
}

namespace
{

// Plane points at random XY positions, `per_m2` points per square meter.
void scatter(
  Cloud & cloud, float x0, float x1, float y0, float y1,
  const std::function<float(float, float)> & f, float per_m2, unsigned seed,
  const std::function<bool(float, float)> & skip = nullptr)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> X(x0, x1), Y(y0, y1);
  const int n = static_cast<int>((x1 - x0) * (y1 - y0) * per_m2);
  for (int k = 0; k < n; ++k) {
    const float x = X(rng), y = Y(rng);
    if (skip && skip(x, y)) {continue;}
    cloud.push_back(pcl::PointXYZ(x, y, f(x, y)));
  }
}

bool has_cell_near(const NavMap & nm, float x, float y, float z, float tol_z = 0.1f)
{
  std::size_t s; NavCelId cid; Eigen::Vector3f bary, hit;
  return nm.locate_navcel({x, y, z}, s, cid, bary, &hit) &&
         std::fabs(hit.x() - x) < 1e-3f && std::fabs(hit.z() - z) < tol_z;
}

uint8_t support_at(const NavMap & nm, float x, float y, float z)
{
  return nm.layer_get<uint8_t>("support", cell_at(nm, x, y, z));
}

}  // namespace

TEST(FromPoints, SpikesDoNotRaiseTheSurface)
{
  Cloud cloud;
  scatter(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f), 200.0f, 5);
  std::mt19937 rng(6);
  std::uniform_real_distribution<float> U(0.0f, 1.0f);
  for (auto & p : cloud.points) {
    if (U(rng) < 0.02f) {p.z += 0.2f * U(rng);}  // 2% spurious returns above the floor
  }
  const NavMap nm = build(cloud);
  expect_clean_mesh(nm);
  for (std::size_t v = 0; v < nm.positions.size(); ++v) {
    EXPECT_NEAR(nm.positions.z[v], 0.0f, 0.02f) << "vertex " << v;
  }
}

TEST(FromPoints, SlopeHeightIsTakenAtTheCellCentre)
{
  // Irregular samples of z = 0.2 x + 0.1 y: vertices must lie on the plane, not at cell tops.
  Cloud cloud;
  scatter(cloud, 0.0f, 10.0f, 0.0f, 10.0f, [](float x, float y) {return 0.2f * x + 0.1f * y;},
    60.0f, 7);
  const NavMap nm = build(cloud);
  expect_clean_mesh(nm);
  // Plane fit is exact with 3+ points; with fewer (border cells) the error is bounded by
  // the slope over half a cell.
  std::vector<uint32_t> min_support(nm.positions.size(), 255);
  for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
    const auto sup = nm.layer_get<uint8_t>("support", cid);
    for (auto v : nm.navcels[cid].v) {
      min_support[v] = std::min<uint32_t>(min_support[v], sup);
    }
  }
  for (std::size_t v = 0; v < nm.positions.size(); ++v) {
    const float expected = 0.2f * nm.positions.x[v] + 0.1f * nm.positions.y[v];
    const float tol = min_support[v] >= 3 ? 1e-3f : (0.2f + 0.1f) * 0.5f * kRes;
    EXPECT_NEAR(nm.positions.z[v], expected, tol) << "vertex " << v;
  }
}

TEST(FromPoints, IrregularDensityStaysOneCleanSheet)
{
  // Dense near the origin, sparse far away, like an accumulated lidar scan.
  Cloud cloud;
  std::mt19937 rng(8);
  std::uniform_real_distribution<float> R(0.0f, 1.0f), A(0.0f, 6.2831853f);
  for (int k = 0; k < 60000; ++k) {
    const float r = 15.0f * std::sqrt(R(rng)) * R(rng);  // density decays with range
    const float a = A(rng);
    cloud.push_back(pcl::PointXYZ(r * std::cos(a), r * std::sin(a), 0.0f));
  }
  const NavMap nm = build(cloud);
  expect_clean_mesh(nm);
  EXPECT_EQ(max_same_height_cover(nm), 1);
  EXPECT_TRUE(has_cell_near(nm, 0.2f, 0.3f, 0.0f));
  EXPECT_TRUE(has_cell_near(nm, 6.0f, 0.0f, 0.0f));
}

TEST(FromPoints, BoxOnTheFloorIsAHole)
{
  Cloud cloud;
  auto in_box = [](float x, float y) {return x > 4.0f && x < 6.0f && y > 4.0f && y < 6.0f;};
  scatter(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f), 100.0f, 9, in_box);
  scatter(cloud, 4.0f, 6.0f, 4.0f, 6.0f, flat(0.4f), 100.0f, 10);  // box top
  for (float z = 0.0f; z < 0.4f; z += 0.05f) {  // box sides
    for (float t = 4.0f; t < 6.0f; t += 0.05f) {
      for (const auto & p : {pcl::PointXYZ(4.0f, t, z), pcl::PointXYZ(6.0f, t, z),
          pcl::PointXYZ(t, 4.0f, z), pcl::PointXYZ(t, 6.0f, z)})
      {
        cloud.push_back(p);
      }
    }
  }
  const NavMap nm = build(cloud);
  expect_clean_mesh(nm);
  // No floor under the box, and the box edge never joins the floor.
  EXPECT_FALSE(has_cell_near(nm, 5.0f, 5.0f, 0.0f));
  const NavCelId floor_cell = cell_at(nm, 1.0f, 1.0f, 0.0f);
  for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
    if (nm.navcel_centroid(cid).z() > 0.2f) {
      EXPECT_FALSE(connected(nm, cid, floor_cell));
      break;
    }
  }
}

TEST(FromPoints, SmallEnclosedHoleIsFilledAndMarked)
{
  Cloud cloud;
  auto gap = [](float x, float y) {return x > 4.0f && x < 5.0f && y > 4.0f && y < 5.0f;};
  scatter(cloud, 0.0f, 10.0f, 0.0f, 10.0f, [](float x, float) {return 0.1f * x;}, 100.0f, 11,
    gap);
  const NavMap nm = build(cloud);  // max_hole_area 1 m2 by default
  expect_clean_mesh(nm);
  EXPECT_EQ(nm.surfaces.size(), 1u);
  ASSERT_TRUE(has_cell_near(nm, 4.5f, 4.5f, 0.45f, 0.02f));
  EXPECT_EQ(support_at(nm, 4.5f, 4.5f, 0.45f), 0u);   // interpolated
  EXPECT_GT(support_at(nm, 1.5f, 1.5f, 0.15f), 0u);   // observed
}

TEST(FromPoints, HoleFillingLimits)
{
  auto plane_with_gap = [](float gx0, float gx1, float gy0, float gy1) {
      Cloud c;
      scatter(c, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f), 100.0f, 12,
        [ = ](float x, float y) {return x > gx0 && x < gx1 && y > gy0 && y < gy1;});
      return c;
    };
  // Larger than max_hole_area: stays open.
  const NavMap big = build(plane_with_gap(3.0f, 6.0f, 3.0f, 6.0f));
  EXPECT_FALSE(has_cell_near(big, 4.5f, 4.5f, 0.0f));

  // Filling disabled.
  auto p = params();
  p.max_hole_area = 0.0f;
  const NavMap off = build(plane_with_gap(4.0f, 5.0f, 4.0f, 5.0f), p);
  EXPECT_FALSE(has_cell_near(off, 4.5f, 4.5f, 0.0f));

  // A notch open to the outside of the data is not a hole.
  const NavMap notch = build(plane_with_gap(-1.0f, 1.0f, 4.0f, 5.0f));
  EXPECT_FALSE(has_cell_near(notch, 0.25f, 4.5f, 0.0f));
}

TEST(FromPoints, PitWithObservedBottomIsNotFilled)
{
  // A 1 x 1 m pit, 1.5 m deep, whose bottom was seen: never bridged at floor level.
  Cloud cloud;
  auto pit = [](float x, float y) {return x > 4.0f && x < 5.0f && y > 4.0f && y < 5.0f;};
  scatter(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f), 100.0f, 13, pit);
  scatter(cloud, 4.0f, 5.0f, 4.0f, 5.0f, flat(-1.5f), 100.0f, 14);
  const NavMap nm = build(cloud);
  expect_clean_mesh(nm);
  EXPECT_FALSE(has_cell_near(nm, 4.5f, 4.5f, 0.0f));
}

TEST(FromPoints, HoleSharedByTwoFloorsIsFilledOnBoth)
{
  Cloud cloud;
  auto gap = [](float x, float y) {return x > 4.0f && x < 5.0f && y > 4.0f && y < 5.0f;};
  scatter(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f), 100.0f, 15, gap);
  scatter(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(3.0f), 100.0f, 16, gap);
  const NavMap nm = build(cloud);
  expect_clean_mesh(nm);
  ASSERT_EQ(nm.surfaces.size(), 2u);
  EXPECT_TRUE(has_cell_near(nm, 4.5f, 4.5f, 0.0f, 0.02f));
  EXPECT_TRUE(has_cell_near(nm, 4.5f, 4.5f, 3.0f, 0.02f));
}

TEST(FromPoints, SupportCountsPointsOfTheWeakestVertex)
{
  // Exactly 4 points per cell: every observed triangle has support 4.
  Cloud cloud;
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {
      // Cells are centred on the first sample (half-cell origin shift).
      for (const auto & [dx, dy] : {std::pair{0.0f, 0.0f}, std::pair{0.1f, 0.0f},
          std::pair{0.0f, 0.1f}, std::pair{0.1f, 0.1f}})
      {
        cloud.push_back(pcl::PointXYZ(0.5f * i + dx, 0.5f * j + dy, 0.0f));
      }
    }
  }
  const NavMap nm = build(cloud);
  ASSERT_FALSE(nm.navcels.empty());
  for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
    EXPECT_EQ(nm.layer_get<uint8_t>("support", cid), 4u);
  }
}

TEST(FromPoints, SlabSupportIsItsUpperFace)
{
  // An upper floor seen from above and from below (its ceiling face), for thin and thick slabs:
  // the vertex must sit on the upper face, not between both faces.
  for (const float thickness : {0.1f, 0.2f, 0.3f}) {
    Cloud cloud;
    scatter(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f), 40.0f, 21);
    scatter(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(2.5f - thickness), 40.0f, 22);  // ceiling face
    scatter(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(2.5f), 40.0f, 23);              // upper face
    const NavMap nm = build(cloud);
    expect_clean_mesh(nm);
    EXPECT_EQ(nm.surfaces.size(), 2u) << thickness;
    float z;
    cell_at(nm, 5.2f, 5.3f, 2.6f, &z);
    // Faces closer than max_dz / 2 cannot be told apart from noise: error up to half the slab.
    const float tol = thickness > 0.5f * params().max_dz ? 0.01f : 0.5f * thickness;
    EXPECT_NEAR(z, 2.5f, tol) << "thickness " << thickness;
    cell_at(nm, 5.2f, 5.3f, 0.1f, &z);
    EXPECT_NEAR(z, 0.0f, 0.01f) << "thickness " << thickness;
  }
}

TEST(FromPoints, SparseUpperFaceOfASlabIsRecoveredByItsNeighbours)
{
  // Upper floor at 2.5 m seen mostly from below: its ceiling face is dense, its top face sparse
  // (about 2 points per cell, many with none). The floor must come out whole, at the top face.
  Cloud cloud;
  scatter(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f), 100.0f, 31);
  scatter(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(2.21f), 100.0f, 32);  // ceiling face
  scatter(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(2.5f), 25.0f, 33);    // sparse top face
  const NavMap nm = build(cloud, params(0.3f));
  expect_clean_mesh(nm);
  ASSERT_EQ(nm.surfaces.size(), 2u);
  float upper_area = 0.0f;
  for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
    if (nm.navcel_centroid(cid).z() > 1.0f) {
      upper_area += nm.navcels[cid].area;
      EXPECT_NEAR(nm.navcel_centroid(cid).z(), 2.5f, 0.02f);
    }
  }
  EXPECT_GT(upper_area, 0.95f * 9.6f * 9.6f);
}

TEST(FromPoints, PotholeIsNotLiftedToTheFloor)
{
  // A 0.6 x 0.6 m depression 0.3 m deep: its points are below the floor, like a slab seen from
  // below, but the floor around has no lower face there, so it must not be lifted.
  Cloud cloud;
  auto pothole = [](float x, float y) {return x > 4.5f && x < 5.1f && y > 4.5f && y < 5.1f;};
  scatter(cloud, 0.0f, 10.0f, 0.0f, 10.0f, flat(0.0f), 100.0f, 41, pothole);
  scatter(cloud, 4.5f, 5.1f, 4.5f, 5.1f, flat(-0.3f), 100.0f, 42);
  const NavMap nm = build(cloud, params(0.3f));
  expect_clean_mesh(nm);
  EXPECT_FALSE(has_cell_near(nm, 4.8f, 4.8f, 0.0f, 0.05f));
}

// ----------------- Clouds with viewpoints -----------------

namespace
{

using VpCloud = pcl::PointCloud<pcl::PointWithViewpoint>;

pcl::PointWithViewpoint vp_point(const Eigen::Vector3f & p, const Eigen::Vector3f & o)
{
  pcl::PointWithViewpoint q;
  q.x = p.x(); q.y = p.y(); q.z = p.z();
  q.vp_x = o.x(); q.vp_y = o.y(); q.vp_z = o.z();
  return q;
}

// Dense samples (0.1 m) of the plane z over a box, each seen from o (or from o + offset per x).
void sample_vp(
  VpCloud & cloud, float x0, float x1, float y0, float y1, float z,
  const std::function<Eigen::Vector3f(float, float)> & origin)
{
  for (float x = x0 + 0.05f; x < x1; x += 0.1f) {
    for (float y = y0 + 0.05f; y < y1; y += 0.1f) {
      cloud.push_back(vp_point({x, y, z}, origin(x, y)));
    }
  }
}

Cloud xyz(const VpCloud & c)
{
  Cloud out;
  for (const auto & p : c) {
    out.push_back({p.x, p.y, p.z});
  }
  return out;
}

NavMap build_vp(
  const VpCloud & cloud, navmap_ros::BuildParams p = params(),
  navmap_ros::ViewpointFilterStats * st = nullptr)
{
  navmap_ros_interfaces::msg::NavMap msg;
  if (st) {navmap_ros::filter_with_viewpoints(cloud, p, st);}
  return navmap_ros::from_points(cloud, msg, p);
}

// Lidar rings over the plane z = 0 from a sensor at o: the real geometry of grazing returns.
void floor_scan(VpCloud & cloud, const Eigen::Vector3f & o, float max_range, unsigned seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<float> N(0.0f, 0.02f);
  for (float el = -15.0f; el <= -1.0f; el += 1.0f) {
    const float e = el * static_cast<float>(M_PI) / 180.0f;
    for (float az = 0.0f; az < 360.0f; az += 0.4f) {
      const float a = az * static_cast<float>(M_PI) / 180.0f;
      const Eigen::Vector3f d(std::cos(e) * std::cos(a), std::cos(e) * std::sin(a), std::sin(e));
      const float t = -o.z() / d.z();
      if (t * std::cos(e) > max_range) {continue;}
      cloud.push_back(vp_point(o + (t + N(rng)) * d, o));
    }
  }
}

}  // namespace

TEST(FromPointsViewpoints, CeilingSeenFromBelowIsDropped)
{
  VpCloud c;
  auto robot = [](float x, float y) {return Eigen::Vector3f(x - 1.0f, y, 0.8f);};
  sample_vp(c, 0, 10, 0, 6, 0.0f, robot);
  sample_vp(c, 0, 10, 0, 6, 2.5f, robot);
  EXPECT_EQ(build(xyz(c)).surfaces.size(), 2u);  // the ceiling is a phantom floor
  navmap_ros::ViewpointFilterStats st;
  const NavMap nm = build_vp(c, params(), &st);
  ASSERT_EQ(nm.surfaces.size(), 1u);
  EXPECT_EQ(st.seen_from_below, 6000u);
  EXPECT_EQ(st.carved, 0u);
  for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
    EXPECT_LT(std::fabs(nm.navcel_centroid(cid).z()), 0.05f);
  }
}

TEST(FromPointsViewpoints, FarCeilingSeenByGrazingRaysIsDropped)
{
  // The upward rings (2.5 to 4.5 degrees) hit a ceiling 1.6 m above the sensor, 20 to 37 m away,
  // over a floor seen from above (by the robot on its way)
  VpCloud c;
  const Eigen::Vector3f o(0.0f, 0.0f, 0.8f);
  for (float x = -39.9f; x < 40.0f; x += 0.2f) {
    for (float y = -39.9f; y < 40.0f; y += 0.2f) {
      c.push_back(vp_point({x, y, 0.0f}, {x - 2.0f, y, 0.8f}));
    }
  }
  const std::size_t floor = c.size();
  for (float el = 2.5f; el <= 4.5f; el += 1.0f) {
    const float e = el * static_cast<float>(M_PI) / 180.0f;
    for (float az = 0.0f; az < 360.0f; az += 0.4f) {
      const float a = az * static_cast<float>(M_PI) / 180.0f;
      const Eigen::Vector3f d(std::cos(e) * std::cos(a), std::cos(e) * std::sin(a), std::sin(e));
      c.push_back(vp_point(o + (1.6f / d.z()) * d, o));
    }
  }
  navmap_ros::ViewpointFilterStats st;
  navmap_ros::filter_with_viewpoints(c, params(), &st);
  // Almost all: a ring crossing a cell's corner over little floor is kept, to be safe
  EXPECT_GE(st.seen_from_below, (c.size() - floor) * 95 / 100);
  EXPECT_LT(st.seen_from_below, c.size() - floor + floor / 1000);
}

TEST(FromPointsViewpoints, CrestSeenFromTheFootOfItsSlopeIsKept)
{
  // A pit (z = 0, x < 5) and a terrace (z = 2, x > 5) with a vertical face between: from the pit,
  // the rays climb to the terrace's edge, which is seen from below with only rock under it
  VpCloud c;
  auto pit = [](float, float y) {return Eigen::Vector3f(1.0f, y, 0.8f);};
  sample_vp(c, 0, 5, 0, 6, 0.0f, pit);
  sample_vp(c, 5, 5.5f, 0, 6, 2.0f, pit);
  navmap_ros::ViewpointFilterStats st;
  const NavMap nm = build_vp(c, params(), &st);
  EXPECT_EQ(st.seen_from_below, 0u);
  EXPECT_NEAR(total_area(nm), total_area(build(xyz(c))), 1e-3f);
}

TEST(FromPointsViewpoints, UpperFloorStandsOnItsTopFace)
{
  // A 0.3 m slab: its top seen from the upper floor, its underside from the lower one.
  VpCloud c;
  sample_vp(c, 0, 10, 0, 6, 0.0f, [](float x, float y) {return Eigen::Vector3f(x - 1, y, 0.8f);});
  sample_vp(c, 0, 10, 0, 6, 2.7f, [](float x, float y) {return Eigen::Vector3f(x - 1, y, 0.8f);});
  // The top is seen sparsely (every other sample), so the faces have similar weight
  VpCloud top;
  sample_vp(top, 0, 10, 0, 6, 3.0f, [](float x, float y) {return Eigen::Vector3f(x - 1, y, 3.8f);});
  for (std::size_t k = 0; k < top.size(); k += 2) {
    c.push_back(top[k]);
  }
  navmap_ros::ViewpointFilterStats st;
  const NavMap nm = build_vp(c, params(), &st);
  ASSERT_EQ(nm.surfaces.size(), 2u);
  EXPECT_EQ(st.seen_from_below, 6000u);
  std::size_t upper = 0;
  for (NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
    if (nm.navcel_centroid(cid).z() > 1.0f) {
      ++upper;
      EXPECT_NEAR(nm.navcel_centroid(cid).z(), 3.0f, 0.02f);
    }
  }
  EXPECT_GT(upper, 0u);
}

TEST(FromPointsViewpoints, TableUndersideStillBlocksTheFloor)
{
  // Floor seen from above; a table top at 0.7 m seen only from below, from the floor's side.
  VpCloud c;
  auto robot = [](float x, float y) {return Eigen::Vector3f(x < 5 ? 1.0f : 9.0f, y, 0.3f);};
  sample_vp(c, 0, 10, 0, 6, 0.0f, robot);
  VpCloud table;
  sample_vp(table, 4, 6, 2, 4, 0.7f, robot);
  c += table;
  navmap_ros::ViewpointFilterStats st;
  const NavMap with = build_vp(c, params(), &st);
  const NavMap without = build(xyz(c));
  EXPECT_EQ(st.seen_from_below, 0u);
  EXPECT_NEAR(total_area(with), total_area(without), 1e-3f);
  EXPECT_LT(total_area(with), 60.0f - 2.0f);  // a hole under the table
}

TEST(FromPointsViewpoints, PersonSeenThroughIsCarved)
{
  // A person (0.4 x 0.4 x 1.7 m) seen in one scan, then the floor behind it seen from far away
  // through where it stood. Its feet left floor points around it.
  VpCloud c;
  const Eigen::Vector3f first(0.0f, 3.0f, 0.8f);
  for (float x = 3.05f; x < 3.45f; x += 0.1f) {
    for (float y = 2.85f; y < 3.25f; y += 0.1f) {
      for (float z = 0.05f; z < 1.7f; z += 0.1f) {
        c.push_back(vp_point({x, y, z}, first));
      }
    }
  }
  // Later scans: the floor, and the upward rings on a wall at x = 8
  for (float ox = 0.0f; ox <= 1.0f; ox += 0.25f) {
    const Eigen::Vector3f o(ox, 3.0f, 0.8f);
    floor_scan(c, o, 8.0f, static_cast<unsigned>(ox * 4 + 1));
    for (float el = 1.0f; el <= 15.0f; el += 1.0f) {
      const float e = el * static_cast<float>(M_PI) / 180.0f;
      for (float az = -30.0f; az <= 30.0f; az += 0.4f) {
        const float a = az * static_cast<float>(M_PI) / 180.0f;
        const Eigen::Vector3f d(std::cos(e) * std::cos(a), std::cos(e) * std::sin(a), std::sin(e));
        c.push_back(vp_point(o + ((8.0f - ox) / d.x()) * d, o));
      }
    }
  }
  const NavMap without = build(xyz(c));
  navmap_ros::ViewpointFilterStats st;
  const NavMap with = build_vp(c, params(), &st);
  EXPECT_GE(st.carved, 4u * 4u * 17u);
  // Without carving the person is an obstacle; carved, the floor there is walkable
  auto floor_at = [](const NavMap & nm, float x, float y) {
      navmap::NavCelId cid;
      float t;
      Eigen::Vector3f hit;
      return nm.raycast({x, y, 3.0f}, {0.0f, 0.0f, -1.0f}, cid, t, hit) && hit.z() < 0.05f;
    };
  EXPECT_TRUE(floor_at(with, 3.25f, 3.05f));
  EXPECT_FALSE(floor_at(without, 3.25f, 3.05f));
}

TEST(FromPointsViewpoints, GrazingRaysDoNotEatTheFloor)
{
  VpCloud c;
  for (int k = 0; k < 5; ++k) {
    floor_scan(c, {2.0f * k, 0.0f, 0.8f}, 25.0f, static_cast<unsigned>(k + 1));
  }
  navmap_ros::ViewpointFilterStats st;
  const NavMap with = build_vp(c, params(), &st);
  EXPECT_LT(st.carved, c.size() / 100);
  EXPECT_LT(st.seen_from_below, c.size() / 1000);  // a few noisy returns
  EXPECT_GT(total_area(with), 0.98f * total_area(build(xyz(c))));
}

TEST(FromPointsViewpoints, RampSeenFromItsFootIsKept)
{
  // A 20 degree ramp seen from its foot: the rays climb, but less than the ramp
  VpCloud c;
  const float s = std::tan(20.0f * static_cast<float>(M_PI) / 180.0f);
  const Eigen::Vector3f o(-1.0f, 3.0f, 0.8f);
  for (float x = 0.05f; x < 8.0f; x += 0.1f) {
    for (float y = 0.05f; y < 6.0f; y += 0.1f) {
      c.push_back(vp_point({x, y, s * x}, o));
    }
  }
  navmap_ros::ViewpointFilterStats st;
  const NavMap with = build_vp(c, params(), &st);
  EXPECT_EQ(st.seen_from_below, 0u);
  EXPECT_EQ(st.carved, 0u);
  EXPECT_NEAR(total_area(with), total_area(build(xyz(c))), 1e-3f);
}

TEST(FromPointsViewpoints, DescendingRaysNeverSeeFromBelow)
{
  // Sparse noisy floor cells (3 points) whose fitted plane tilts more than the rays
  VpCloud c;
  std::mt19937 rng(3);
  std::uniform_real_distribution<float> U(-0.2f, 0.2f), N(-0.03f, 0.03f);
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 12; ++j) {
      for (int k = 0; k < 3; ++k) {
        const Eigen::Vector3f p(i * kRes + 0.25f + U(rng), j * kRes + 0.25f + U(rng), N(rng));
        c.push_back(vp_point(p, {p.x() - 15.0f, p.y(), 0.8f}));  // 3 degrees down
      }
    }
  }
  navmap_ros::ViewpointFilterStats st;
  navmap_ros::filter_with_viewpoints(c, params(), &st);
  EXPECT_EQ(st.seen_from_below, 0u);
}

TEST(FromPointsViewpoints, WallSeenByUpwardRingsIsKept)
{
  // A wall from 1.2 to 3 m (its foot hidden), seen from a robot on the floor in front of it
  VpCloud c;
  auto robot = [](float, float y) {return Eigen::Vector3f(1.0f, y, 0.8f);};
  sample_vp(c, 0, 10, 0, 6, 0.0f, robot);
  for (float y = 0.05f; y < 6.0f; y += 0.1f) {
    for (float z = 1.25f; z < 3.0f; z += 0.1f) {
      c.push_back(vp_point({5.02f, y, z}, robot(0, y)));
    }
  }
  navmap_ros::ViewpointFilterStats st;
  build_vp(c, params(), &st);
  EXPECT_EQ(st.seen_from_below, 0u);
  EXPECT_EQ(st.carved, 0u);
}

TEST(FromPointsViewpoints, FilterIsDeterministic)
{
  // Many voxels (the index grows) and rays crossing the same voxels from several threads
  VpCloud c;
  for (int k = 0; k < 6; ++k) {
    floor_scan(c, {3.0f * k, 1.0f * k, 0.8f}, 25.0f, static_cast<unsigned>(k + 1));
  }
  for (float x = 5.05f; x < 5.45f; x += 0.1f) {
    for (float z = 0.05f; z < 1.7f; z += 0.1f) {
      c.push_back(vp_point({x, 2.0f, z}, {0.0f, 2.0f, 0.8f}));
    }
  }
  navmap_ros::ViewpointFilterStats a, b;
  const Cloud fa = navmap_ros::filter_with_viewpoints(c, params(), &a);
  const Cloud fb = navmap_ros::filter_with_viewpoints(c, params(), &b);
  EXPECT_EQ(a.carved, b.carved);
  EXPECT_EQ(a.seen_from_below, b.seen_from_below);
  ASSERT_EQ(fa.size(), fb.size());
  for (std::size_t k = 0; k < fa.size(); ++k) {
    ASSERT_EQ(fa[k].x, fb[k].x);
    ASSERT_EQ(fa[k].z, fb[k].z);
  }
}

TEST(FromPointsViewpoints, DisabledFiltersMatchThePlainCloud)
{
  VpCloud c;
  auto robot = [](float x, float y) {return Eigen::Vector3f(x - 1.0f, y, 0.8f);};
  sample_vp(c, 0, 10, 0, 6, 0.0f, robot);
  sample_vp(c, 0, 10, 0, 6, 2.5f, robot);
  auto p = params();
  p.carve_free_space = false;
  p.drop_seen_from_below = false;
  navmap_ros::ViewpointFilterStats st;
  const NavMap a = build_vp(c, p, &st);
  const NavMap b = build(xyz(c));
  EXPECT_EQ(st.carved + st.seen_from_below, 0u);
  EXPECT_EQ(st.input, c.size());
  EXPECT_EQ(a.navcels.size(), b.navcels.size());
  EXPECT_EQ(a.surfaces.size(), b.surfaces.size());
}

TEST(FromPointsViewpoints, PointsWithoutViewpointAreKept)
{
  VpCloud c;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  sample_vp(c, 0, 10, 0, 6, 0.0f, [&](float, float) {return Eigen::Vector3f(nan, nan, nan);});
  sample_vp(c, 0, 10, 0, 6, 2.5f, [&](float, float) {return Eigen::Vector3f(nan, nan, nan);});
  navmap_ros::ViewpointFilterStats st;
  const NavMap nm = build_vp(c, params(), &st);
  EXPECT_EQ(st.carved + st.seen_from_below, 0u);
  EXPECT_EQ(nm.surfaces.size(), 2u);
}

TEST(FromPointsViewpoints, EmptyAndDegenerateInputs)
{
  navmap_ros::ViewpointFilterStats st;
  EXPECT_TRUE(build_vp(VpCloud(), params(), &st).navcels.empty());
  EXPECT_EQ(st.input, 0u);
  VpCloud one;
  one.push_back(vp_point({0, 0, 0}, {0, 0, 0}));  // viewpoint on the point
  EXPECT_TRUE(build_vp(one, params(), &st).navcels.empty());
  EXPECT_EQ(st.carved, 0u);
  auto p = params();
  p.resolution = 0.0f;
  EXPECT_TRUE(build_vp(one, p, &st).navcels.empty());
}
