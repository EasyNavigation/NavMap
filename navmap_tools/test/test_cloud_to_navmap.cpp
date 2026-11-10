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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <random>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "navmap_tools/cloud_to_navmap.hpp"
#include "navmap_tools/pcd_stream.hpp"
#include "navmap_tools/voxel_cloud.hpp"

namespace
{
struct Pt {float x, y, z; std::uint8_t r, g, b;};

std::string tmp(const std::string & name)
{
  return (std::filesystem::temp_directory_path() / ("navmap_tools_" + name)).string();
}

// height > 1: an organised cloud of pts.size() / height columns
void write_pcd(
  const std::string & path, const std::vector<Pt> & pts, const std::string & data,
  bool colour, std::size_t height = 1)
{
  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  f.precision(9);   // enough digits for a float to come back bit for bit (the packed colour is one)
  f << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z" << (colour ? " rgb" : "") << "\nSIZE 4 4 4"
    << (colour ? " 4" : "") << "\nTYPE F F F" << (colour ? " F" : "") << "\nCOUNT 1 1 1"
    << (colour ? " 1" : "") << "\nWIDTH " << pts.size() / height <<
    "\nHEIGHT " << height << "\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "
    << pts.size() << "\nDATA " << data << "\n";
  for (const auto & p : pts) {
    const std::uint32_t u = (static_cast<std::uint32_t>(p.r) <<
      16) | (static_cast<std::uint32_t>(p.g) << 8) | p.b;
    float rgbf;
    std::memcpy(&rgbf, &u, 4);
    if (data == "binary") {
      f.write(reinterpret_cast<const char *>(&p.x), 4);
      f.write(reinterpret_cast<const char *>(&p.y), 4);
      f.write(reinterpret_cast<const char *>(&p.z), 4);
      if (colour) {f.write(reinterpret_cast<const char *>(&rgbf), 4);}
    } else {
      f << p.x << " " << p.y << " " << p.z;
      if (colour) {f << " " << rgbf;}
      f << "\n";
    }
  }
}

// A 30 x 30 m floor (red) at z = 0 and a 15 x 30 m mezzanine (green) at z = 3 over half of it, with
// `spacing` between points, plus a little noise
std::vector<Pt> two_floors(float spacing, unsigned seed = 1)
{
  std::mt19937 g(seed);
  std::normal_distribution<float> noise(0.0f, 0.01f);
  std::vector<Pt> pts;
  for (float x = 0; x < 30.0f; x += spacing) {
    for (float y = 0; y < 30.0f; y += spacing) {
      pts.push_back({x, y, noise(g), 200, 20, 20});
      if (x >= 15.0f) {pts.push_back({x, y, 3.0f + noise(g), 20, 200, 20});}
    }
  }
  return pts;
}

std::set<int> levels(const navmap_ros_interfaces::msg::NavMap & m)
{
  std::set<int> z;
  for (const float v : m.positions_z) {
    z.insert(static_cast<int>(std::lround(v)));
  }
  return z;
}
}  // namespace

TEST(PcdStream, BinaryAndAsciiGiveTheSamePointsAndColours)
{
  const auto pts = two_floors(1.0f);
  for (const bool colour : {true, false}) {
    for (const std::string data : {"binary", "ascii"}) {
      const auto path = tmp("rt_" + data + (colour ? "_c" : "_n") + ".pcd");
      write_pcd(path, pts, data, colour);
      const auto info = navmap_tools::pcd_info(path);
      EXPECT_EQ(info.points, pts.size());
      EXPECT_EQ(info.has_color, colour);
      EXPECT_TRUE(info.streamed);
      std::size_t n = 0;
      bool ok = true;
      navmap_tools::read_pcd(path, [&](const navmap_tools::PcdChunk & c) {
          for (std::size_t k = 0; k < c.size(); ++k, ++n) {
            ok = ok && std::abs(c.xyz[3 * k] - pts[n].x) < 1e-3f &&
            std::abs(c.xyz[3 * k + 2] - pts[n].z) < 1e-3f;
            if (colour) {
              ok = ok && c.rgb[3 * k] == pts[n].r && c.rgb[3 * k + 1] == pts[n].g &&
              c.rgb[3 * k + 2] == pts[n].b;
            }
          }
        }, 100);          // many small blocks
      EXPECT_EQ(n, pts.size()) << data;
      EXPECT_TRUE(ok) << data << (colour ? " colour" : "");
      std::filesystem::remove(path);
    }
  }
}

TEST(PcdStream, ABrokenFileSaysWhy)
{
  const auto path = tmp("broken.pcd");
  {std::ofstream f(path); f << "this is not a pcd\n";}
  EXPECT_THROW(navmap_tools::pcd_info(path), std::runtime_error);
  EXPECT_THROW(navmap_tools::pcd_info("/nonexistent/x.pcd"), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(VoxelCloud, ReducesPointsToVoxelsAndKeepsThePlane)
{
  navmap_tools::VoxelCloud v(0.5f, 0.125f);
  std::size_t n = 0;
  for (int i = 0; i < 400; ++i) {
    for (int j = 0; j < 400; ++j, ++n) {
      const float x = 0.05f * static_cast<float>(i), y = 0.05f * static_cast<float>(j);
      v.add(x, y, 0.3f + 0.1f * x, nullptr);   // a ramp
    }
  }
  const auto c = v.centroids();
  EXPECT_EQ(n, 400u * 400u);
  // ~ (20 / 0.5)^2 columns, a few z bins on a slope
  EXPECT_LT(c.size(), 20u * 2 * 20 * 2 * 3);
  EXPECT_GT(c.size(), 1000u);
  double worst = 0;
  for (std::size_t k = 0; k < c.size(); ++k) {   // every centroid is still on the ramp
    worst = std::max<double>(worst, std::abs(c.xyz[3 * k + 2] - (0.3f + 0.1f * c.xyz[3 * k])));
  }
  EXPECT_LT(worst, 0.03);
  std::uint32_t total = 0;
  for (const auto k : c.count) {
    total += k;
                                           }
  EXPECT_EQ(total, n);                              // no point is lost, only merged
  EXPECT_TRUE(c.rgb.empty());
}

TEST(VoxelCloud, DropsIsolatedNoiseWhenAsked)
{
  navmap_tools::VoxelCloud v(0.5f, 0.125f);
  for (int i = 0; i < 100; ++i) {
    v.add(1.1f, 1.1f, 0.0f, nullptr);
                                                                  }
  v.add(8.0f, 8.0f, 5.0f, nullptr);                 // a lone point
  EXPECT_EQ(v.centroids(1).size(), 2u);
  EXPECT_EQ(v.centroids(2).size(), 1u);
}

TEST(CloudToNavmap, TwoFloorsKeepBothAndTheirColours)
{
  const auto path = tmp("two.pcd");
  write_pcd(path, two_floors(0.25f), "binary", true);
  navmap_tools::Options opt;
  opt.resolution = 1.0f;
  opt.min_clearance = 1.0f;
  navmap_ros_interfaces::msg::NavMap m;
  const auto rep = navmap_tools::cloud_to_navmap({path}, opt, m);
  EXPECT_FALSE(rep.voxelized);
  EXPECT_TRUE(rep.coloured);
  const auto z = levels(m);
  EXPECT_TRUE(z.count(0) && z.count(3)) << "both floors must be meshed";
  ASSERT_EQ(m.colors_r.size(), m.positions_z.size());
  std::size_t red0 = 0, n0 = 0, green3 = 0, n3 = 0;
  for (std::size_t v = 0; v < m.positions_z.size(); ++v) {
    if (std::abs(m.positions_z[v]) < 0.2f) {
      ++n0; red0 += m.colors_r[v] > 150 && m.colors_g[v] < 80;
    }
    if (std::abs(m.positions_z[v] - 3.0f) < 0.2f) {
      ++n3; green3 += m.colors_g[v] > 150 && m.colors_r[v] < 80;
    }
  }
  EXPECT_GT(n0, 100u);
  EXPECT_GT(n3, 100u);
  EXPECT_EQ(red0, n0) << "the lower floor is red";
  EXPECT_EQ(green3, n3) << "the mezzanine is green, not the colour of the floor below it";
  EXPECT_EQ(m.header.frame_id, "map");
  std::filesystem::remove(path);
}

TEST(CloudToNavmap, TheVoxelReductionGivesTheSameMapAsAllThePoints)
{
  const auto path = tmp("vox.pcd");
  write_pcd(path, two_floors(0.1f), "binary", true);      // 200k points for 1 m cells
  navmap_tools::Options opt;
  opt.resolution = 1.0f;
  navmap_ros_interfaces::msg::NavMap raw, red;
  opt.voxel = 0.0f;
  const auto r0 = navmap_tools::cloud_to_navmap({path}, opt, raw);
  opt.voxel = 0.25f;
  const auto r1 = navmap_tools::cloud_to_navmap({path}, opt, red);
  EXPECT_FALSE(r0.voxelized);
  EXPECT_TRUE(r1.voxelized);
  EXPECT_LT(r1.points_meshed * 3, r0.points_meshed) << "far fewer points to mesh";
  EXPECT_EQ(levels(raw), levels(red));
  const double dv = std::abs(static_cast<double>(raw.positions_x.size()) - red.positions_x.size());
  EXPECT_LT(dv, 0.05 * raw.positions_x.size());
  const double dt = std::abs(static_cast<double>(raw.navcels_v0.size()) - red.navcels_v0.size());
  EXPECT_LT(dt, 0.05 * raw.navcels_v0.size());
  // The mesher's lattice starts at the lowest point, which the reduction moves by a few cm: the
  // vertices are not the same, but both floors must be as flat and as high as with every point
  for (const auto * m : {&raw, &red}) {
    double worst = 0;
    std::size_t bad = 0;
    for (std::size_t v = 0; v < m->positions_z.size(); ++v) {
      const float z = m->positions_z[v];
      const double e = std::min(std::abs(z), std::abs(z - 3.0f));
      worst = std::max(worst, e);
      if (e > 0.05 && bad++ < 3) {
        std::printf("   off-floor vertex %s at (%.2f, %.2f, %.2f)\n", m == &raw ? "raw" : "red",
          m->positions_x[v], m->positions_y[v], z);
      }
    }
    std::printf("   %s: %zu vertices, %zu off the floors (worst %.2f)\n", m == &raw ? "raw" : "red",
      m->positions_z.size(), bad, worst);
    EXPECT_LT(worst, 0.05);
  }
  std::filesystem::remove(path);
}

TEST(CloudToNavmap, TooSparseDataIsReportedWithTheResolutionItHolds)
{
  const auto path = tmp("sparse.pcd");
  write_pcd(path, two_floors(0.9f), "binary", false);      // ~1 point per 1 m cell
  navmap_tools::Options opt;
  opt.resolution = 1.0f;
  navmap_ros_interfaces::msg::NavMap m;
  const auto rep = navmap_tools::cloud_to_navmap({path}, opt, m);
  EXPECT_LT(rep.median_points_per_cell, 3.0);
  EXPECT_GT(rep.supported_resolution, 1.2) << "it must suggest a coarser resolution";
  ASSERT_FALSE(rep.warnings.empty());
  EXPECT_NE(rep.warnings[0].find("points per cell"), std::string::npos);
  std::filesystem::remove(path);
}

TEST(CloudToNavmap, CropAndHeightWindowSelectWhatIsUsed)
{
  const auto path = tmp("crop.pcd");
  write_pcd(path, two_floors(0.25f), "binary", false);
  navmap_tools::Options opt;
  opt.resolution = 1.0f;
  opt.zmax = 1.0;                                   // the mezzanine is a ceiling: left out
  navmap_ros_interfaces::msg::NavMap m;
  navmap_tools::cloud_to_navmap({path}, opt, m);
  EXPECT_EQ(levels(m), (std::set<int>{0}));
  opt.zmax = 1e9;
  opt.crop = true; opt.xmin = 0; opt.ymin = 0; opt.xmax = 10; opt.ymax = 10;
  navmap_ros_interfaces::msg::NavMap c;
  navmap_tools::cloud_to_navmap({path}, opt, c);
  float xmax = 0, ymax = 0;
  for (std::size_t v = 0; v < c.positions_x.size(); ++v) {
    xmax = std::max(xmax, c.positions_x[v]); ymax = std::max(ymax, c.positions_y[v]);
  }
  EXPECT_LT(xmax, 11.0f);
  EXPECT_LT(ymax, 11.0f);
  std::filesystem::remove(path);
}

TEST(CloudToNavmap, SeveralFilesAreJoined)
{
  const auto a = tmp("a.pcd"), b = tmp("b.pcd");
  auto pts = two_floors(0.25f);
  std::vector<Pt> first, second;
  for (const auto & p : pts) {
    (p.y < 15.0f ? first : second).push_back(p);
                                                                          }
  write_pcd(a, first, "binary", false);
  write_pcd(b, second, "ascii", false);
  navmap_tools::Options opt;
  opt.resolution = 1.0f;
  navmap_ros_interfaces::msg::NavMap m;
  const auto rep = navmap_tools::cloud_to_navmap({a, b}, opt, m);
  EXPECT_EQ(rep.points_in_files, pts.size());
  float ymin = 1e9f, ymax = -1e9f;
  for (const float y : m.positions_y) {
    ymin = std::min(ymin, y); ymax = std::max(ymax, y);
                                                                                          }
  EXPECT_LT(ymin, 2.0f);
  EXPECT_GT(ymax, 28.0f);
  std::filesystem::remove(a);
  std::filesystem::remove(b);
}

TEST(CloudToNavmap, NothingToMeshSaysSo)
{
  const auto path = tmp("none.pcd");
  write_pcd(path, two_floors(0.5f), "binary", false);
  navmap_tools::Options opt;
  opt.crop = true; opt.xmin = 500; opt.ymin = 500; opt.xmax = 600; opt.ymax = 600;
  navmap_ros_interfaces::msg::NavMap m;
  EXPECT_THROW(navmap_tools::cloud_to_navmap({path}, opt, m), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(CloudToNavmap, RecenterPutsTheMiddleOfTheMapAtTheOrigin)
{
  const auto path = tmp("recenter.pcd");
  auto pts = two_floors(0.25f);
  for (auto & p : pts) {
    p.x += 500.0f; p.y += 1200.0f;
                                                      }      // far from the origin
  write_pcd(path, pts, "binary", false);
  navmap_tools::Options opt;
  opt.resolution = 1.0f;
  opt.recenter = true;
  navmap_ros_interfaces::msg::NavMap m;
  const auto rep = navmap_tools::cloud_to_navmap({path}, opt, m);
  float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f;
  for (std::size_t v = 0; v < m.positions_x.size(); ++v) {
    x0 = std::min(x0, m.positions_x[v]); x1 = std::max(x1, m.positions_x[v]);
    y0 = std::min(y0, m.positions_y[v]); y1 = std::max(y1, m.positions_y[v]);
  }
  EXPECT_NEAR(0.5 * (x0 + x1), 0.0, 1.5);
  EXPECT_NEAR(0.5 * (y0 + y1), 0.0, 1.5);
  EXPECT_NEAR(rep.offset_x, -515.0, 1.5);
  EXPECT_NEAR(rep.offset_y, -1215.0, 1.5);
  std::filesystem::remove(path);
}

namespace
{
// a flat 30 x 30 m floor with spacing 0.25 and a rectangular hole [x0, x1] x [y0, y1]
// with no points
std::vector<Pt> floor_with_hole(float x0, float x1, float y0, float y1)
{
  std::vector<Pt> pts;
  for (float x = 0; x < 30.0f; x += 0.25f) {
    for (float y = 0; y < 30.0f; y += 0.25f) {
      if (x >= x0 && x <= x1 && y >= y0 && y <= y1) {continue;}
      pts.push_back({x, y, 0.0f, 128, 128, 128});
    }
  }
  return pts;
}

// triangles whose centroid is in the window
std::size_t triangles_in(
  const navmap_ros_interfaces::msg::NavMap & m, float x0, float x1, float y0, float y1)
{
  std::size_t n = 0;
  for (std::size_t t = 0; t < m.navcels_v0.size(); ++t) {
    const auto a = m.navcels_v0[t], b = m.navcels_v1[t], c = m.navcels_v2[t];
    const float cx = (m.positions_x[a] + m.positions_x[b] + m.positions_x[c]) / 3.0f;
    const float cy = (m.positions_y[a] + m.positions_y[b] + m.positions_y[c]) / 3.0f;
    n += cx > x0 && cx<x1 && cy> y0 && cy < y1;
  }
  return n;
}
}  // namespace

TEST(CloudToNavmap, ASmallHoleInTheFloorIsClosedAndALargeOneIsNot)
{
  const auto path = tmp("holes.pcd");
  auto pts = floor_with_hole(10.0f, 12.0f, 10.0f, 13.0f);   // 2 x 3 m: narrower than 2.4 m
  pts.erase(std::remove_if(pts.begin(), pts.end(), [](const Pt & p) {
      return p.x >= 20.0f && p.x <= 27.0f && p.y >= 20.0f && p.y <= 27.0f;     // 7 x 7 m
      }), pts.end());
  write_pcd(path, pts, "binary", true);
  navmap_tools::Options opt;
  opt.resolution = 0.5f;
  opt.fill_holes = 0.0f;
  navmap_ros_interfaces::msg::NavMap open;
  navmap_tools::cloud_to_navmap({path}, opt, open);
  EXPECT_EQ(triangles_in(open, 10.4f, 11.6f, 10.4f, 12.6f), 0u) << "the hole is there";
  opt.fill_holes = 2.4f;
  navmap_ros_interfaces::msg::NavMap closed;
  const auto rep = navmap_tools::cloud_to_navmap({path}, opt, closed);
  EXPECT_GT(rep.filled_cells, 0u);
  EXPECT_GT(triangles_in(closed, 10.4f, 11.6f, 10.4f, 12.6f), 0u) << "the small hole is closed";
  EXPECT_EQ(triangles_in(closed, 22.0f, 25.0f, 22.0f, 25.0f), 0u) << "the 7 m hole is a real void";
  std::filesystem::remove(path);
}

TEST(PcdStream, AnAsciiRgbOfTypeFWrittenAsAnIntegerIsThePackedValue)
{
  // PCL's ASCII writer stores a packed rgb of TYPE F as its uint32 value (pure red: 16711680, and
  // with alpha 255: 4294901760); a printed float is the float whose bits are the colour
  const auto path = tmp("rgb_int.pcd");
  {
    std::ofstream f(path);
    f << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\n"
      "COUNT 1 1 1 1\nWIDTH 3\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 3\nDATA ascii\n"
      "0 0 0 16711680\n1 0 0 4278255360\n2 0 0 9.183409486e-41\n";
  }
  std::vector<std::uint8_t> rgb;
  navmap_tools::read_pcd(path, [&](const navmap_tools::PcdChunk & c) {
      rgb.insert(rgb.end(), c.rgb.begin(), c.rgb.end());
    });
  ASSERT_EQ(rgb.size(), 9u);
  EXPECT_EQ((std::vector<int>{rgb[0], rgb[1], rgb[2]}), (std::vector<int>{255, 0, 0}));
  EXPECT_EQ((std::vector<int>{rgb[3], rgb[4], rgb[5]}), (std::vector<int>{0, 255, 0}));
  // 9.183409486e-41 is the float of the bits 0x0000ffff: blue and green
  EXPECT_EQ((std::vector<int>{rgb[6], rgb[7], rgb[8]}), (std::vector<int>{0, 255, 255}));
  std::filesystem::remove(path);
}

TEST(VoxelCloud, TheColourOfAVoxelIsTheMeanOfItsPointsWithColour)
{
  navmap_tools::VoxelCloud v(1.0f, 1.0f);
  const std::uint8_t red[3] = {200, 10, 10};
  v.add(0.2f, 0.2f, 0.2f, red);
  v.add(0.4f, 0.4f, 0.2f, red);
  v.add(0.6f, 0.6f, 0.2f, nullptr);   // same voxel, no colour: it must not darken it
  v.add(5.5f, 5.5f, 0.2f, nullptr);   // a voxel with no colour at all
  const auto c = v.centroids();
  ASSERT_EQ(c.size(), 2u);
  ASSERT_EQ(c.rgb.size(), 6u);
  for (std::size_t k = 0; k < c.size(); ++k) {
    const bool first = c.xyz[3 * k] < 1.0f;
    EXPECT_EQ(c.rgb[3 * k], first ? 200 : navmap_tools::kNoColour);
    EXPECT_EQ(c.rgb[3 * k + 1], first ? 10 : navmap_tools::kNoColour);
    EXPECT_EQ(c.count[k], first ? 3u : 1u);
  }
}

TEST(VoxelCloud, ACloudFarFromTheOriginIsKept)
{
  // UTM coordinates: 440 km from (0, 0) is beyond 2^20 voxels of 0.25 m, but the index is
  // relative to the first point
  navmap_tools::VoxelCloud v(0.25f, 0.125f);
  for (int i = 0; i < 100; ++i) {
    EXPECT_TRUE(v.add(440000.0f + 0.1f * static_cast<float>(i), 4470000.0f, 650.0f, nullptr));
  }
  EXPECT_EQ(v.dropped(), 0u);
  const auto c = v.centroids();
  ASSERT_GT(c.size(), 0u);
  EXPECT_NEAR(c.xyz[0], 440005.0f, 6.0f);
  EXPECT_NEAR(c.xyz[1], 4470000.0f, 1.0f);
  EXPECT_NEAR(c.xyz[2], 650.0f, 0.2f);
}

TEST(CloudToNavmap, FilesWithAndWithoutColourKeepEachPointsColour)
{
  // a floor: the half x < 15 m coloured (red) in one file, the other half without colour in
  // another, in both orders and with and without the voxel reduction. Before, the colours of the
  // raw path no longer lined up with the points (read past the end, or put on the wrong points)
  std::vector<Pt> red, plain;
  for (float x = 0; x < 30.0f; x += 0.25f) {
    for (float y = 0; y < 20.0f; y += 0.25f) {
      (x < 15.0f ? red : plain).push_back({x, y, 0.0f, 220, 20, 20});
    }
  }
  const auto a = tmp("mixed_red.pcd"), b = tmp("mixed_plain.pcd");
  write_pcd(a, red, "binary", true);
  write_pcd(b, plain, "binary", false);
  for (const bool red_first : {true, false}) {
    for (const float voxel : {0.0f, 0.25f}) {
      navmap_tools::Options opt;
      opt.resolution = 1.0f;
      opt.voxel = voxel;
      navmap_ros_interfaces::msg::NavMap m;
      const auto rep = navmap_tools::cloud_to_navmap(
        red_first ? std::vector<std::string>{a, b} : std::vector<std::string>{b, a}, opt, m);
      const std::string what = std::string(red_first ? "red first" : "plain first") +
        (voxel > 0 ? ", voxels" : ", raw");
      EXPECT_TRUE(rep.coloured) << what;
      ASSERT_EQ(m.colors_r.size(), m.positions_x.size()) << what;
      std::size_t n_red = 0, ok_red = 0, n_plain = 0, ok_plain = 0;
      for (std::size_t v = 0; v < m.positions_x.size(); ++v) {
        if (m.positions_x[v] < 13.0f) {
          ++n_red; ok_red += m.colors_r[v] == 220 && m.colors_g[v] == 20;
        } else if (m.positions_x[v] > 17.0f) {
          ++n_plain;
          ok_plain += m.colors_r[v] == navmap_tools::kNoColour &&
            m.colors_g[v] == navmap_tools::kNoColour;
        }
      }
      EXPECT_GT(n_red, 100u) << what;
      EXPECT_GT(n_plain, 100u) << what;
      EXPECT_EQ(ok_red, n_red) << what << ": the coloured half keeps its colour";
      EXPECT_EQ(ok_plain, n_plain) << what << ": the half without colour is grey";
    }
  }
  std::filesystem::remove(a);
  std::filesystem::remove(b);
}

TEST(CloudToNavmap, AnOrganisedCloudWithPointOptionsIsMeshedAsPointsAndTheyApply)
{
  // a 30 x 30 m grid, organised (60 rows), far from the origin
  std::vector<Pt> grid;
  for (int j = 0; j < 60; ++j) {
    for (int i = 0; i < 60; ++i) {
      grid.push_back({500.0f + 0.5f * static_cast<float>(i),
          1200.0f + 0.5f * static_cast<float>(j), 0.0f, 0, 0, 0});
    }
  }
  const auto path = tmp("organised.pcd");
  write_pcd(path, grid, "binary", false, 60);
  navmap_tools::Options opt;
  opt.resolution = 1.0f;
  navmap_ros_interfaces::msg::NavMap whole;
  EXPECT_TRUE(navmap_tools::cloud_to_navmap({path}, opt, whole).organised)
    << "without options, the grid mesher";
  EXPECT_EQ(navmap_tools::options_the_grid_mesher_ignores(opt), "");

  opt.crop = true; opt.xmin = 500; opt.ymin = 1200; opt.xmax = 510; opt.ymax = 1210;
  opt.recenter = true;
  EXPECT_EQ(navmap_tools::options_the_grid_mesher_ignores(opt), "crop, recenter");
  navmap_ros_interfaces::msg::NavMap m;
  const auto rep = navmap_tools::cloud_to_navmap({path}, opt, m);
  EXPECT_FALSE(rep.organised);
  float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f;
  for (std::size_t v = 0; v < m.positions_x.size(); ++v) {
    x0 = std::min(x0, m.positions_x[v]); x1 = std::max(x1, m.positions_x[v]);
    y0 = std::min(y0, m.positions_y[v]); y1 = std::max(y1, m.positions_y[v]);
  }
  EXPECT_LT(x1 - x0, 11.5f) << "cropped to 10 m";
  EXPECT_LT(y1 - y0, 11.5f);
  EXPECT_NEAR(0.5 * (x0 + x1), 0.0, 1.5) << "recentred";
  EXPECT_NEAR(rep.offset_x, -505.0, 1.5);
  EXPECT_NEAR(rep.offset_y, -1205.0, 1.5);
  std::filesystem::remove(path);
}

TEST(CloudToNavmap, HolesAreLeftOpenUnlessAskedFor)
{
  EXPECT_EQ(navmap_tools::Options().fill_holes, 0.0f);
}
