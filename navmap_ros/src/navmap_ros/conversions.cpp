// Copyright 2025 Intelligent Robotics Lab
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

#include "navmap_ros/conversions.hpp"

#include <algorithm>
#include <cmath>
#include <cassert>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include <numeric>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "geometry_msgs/msg/pose.hpp"
#include <std_msgs/msg/header.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "navmap_ros_interfaces/msg/nav_map.hpp"
#include "navmap_ros_interfaces/msg/nav_map_layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "navmap_core/Geometry.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

#include "pcl/common/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"
#include "pcl/kdtree/kdtree_flann.h"

namespace navmap_ros
{

using navmap_ros_interfaces::msg::NavMap;
using navmap_ros_interfaces::msg::NavMapLayer;
using navmap_ros_interfaces::msg::NavMapSurface;

// ----------------- Helpers (encoding) -----------------

static inline uint8_t occ_to_u8(int8_t v)
{
  if (v < 0) {return 255u;}
  if (v >= 100) {return 254u;}
  return static_cast<uint8_t>(std::lround((v / 100.0) * 254.0));
}

static inline int8_t u8_to_occ(uint8_t u)
{
  if (u == 255u) {return -1;}
  // round back to 0..100
  return static_cast<int8_t>(std::lround((u / 254.0) * 100.0));
}

// Given grid dims (W,H) and pattern=0, the two triangle indices for cell (i,j):
// tri0=(i,j)->(i+1,j)->(i+1,j+1), tri1=(i,j)->(i+1,j+1)->(i,j+1)
static inline navmap::NavCelId tri_index_for_cell(uint32_t i, uint32_t j, uint32_t W)
{
  return static_cast<navmap::NavCelId>((j * W + i) * 2);
}

// ----------------- NavMap <-> ROS message -----------------

NavMap to_msg(const navmap::NavMap & nm)
{
  NavMap out;

  // positions
  out.positions_x.assign(nm.positions.x.begin(), nm.positions.x.end());
  out.positions_y.assign(nm.positions.y.begin(), nm.positions.y.end());
  out.positions_z.assign(nm.positions.z.begin(), nm.positions.z.end());

  // colors (optional)
  if (nm.colors.has_value()) {
    out.has_vertex_rgba = true;
    out.colors_r = nm.colors->r;
    out.colors_g = nm.colors->g;
    out.colors_b = nm.colors->b;
    out.colors_a = nm.colors->a;
  } else {
    out.has_vertex_rgba = false;
  }

  // triangles
  out.navcels_v0.reserve(nm.navcels.size());
  out.navcels_v1.reserve(nm.navcels.size());
  out.navcels_v2.reserve(nm.navcels.size());
  for (const auto & c : nm.navcels) {
    out.navcels_v0.push_back(c.v[0]);
    out.navcels_v1.push_back(c.v[1]);
    out.navcels_v2.push_back(c.v[2]);
  }

  // surfaces
  out.surfaces.reserve(nm.surfaces.size());
  for (const auto & s : nm.surfaces) {
    NavMapSurface smsg;
    smsg.frame_id = s.frame_id;
    smsg.navcels.assign(s.navcels.begin(), s.navcels.end());
    out.surfaces.push_back(std::move(smsg));
  }

  // layers (per-NavCel)
  for (const auto & lname : nm.layers.list()) {
    auto base = nm.layers.get(lname);
    if (!base) {continue;}

    NavMapLayer lmsg;
    lmsg.name = lname;
    lmsg.type = static_cast<uint8_t>(base->type());

    switch (base->type()) {
      case navmap::LayerType::U8: {
          auto v = std::dynamic_pointer_cast<navmap::LayerView<uint8_t>>(base);
          lmsg.data_u8 = v->data();
        } break;
      case navmap::LayerType::F32: {
          auto v = std::dynamic_pointer_cast<navmap::LayerView<float>>(base);
          lmsg.data_f32 = v->data();
        } break;
      case navmap::LayerType::F64: {
          auto v = std::dynamic_pointer_cast<navmap::LayerView<double>>(base);
          lmsg.data_f64 = v->data();
        } break;
    }
    out.layers.push_back(std::move(lmsg));
  }

  return out;
}

navmap::NavMap from_msg(const NavMap & msg)
{
  navmap::NavMap nm;

  // positions
  nm.positions.x.assign(msg.positions_x.begin(), msg.positions_x.end());
  nm.positions.y.assign(msg.positions_y.begin(), msg.positions_y.end());
  nm.positions.z.assign(msg.positions_z.begin(), msg.positions_z.end());

  // colors
  if (msg.has_vertex_rgba) {
    navmap::Colors col;
    col.r = msg.colors_r;
    col.g = msg.colors_g;
    col.b = msg.colors_b;
    col.a = msg.colors_a;
    nm.colors = std::move(col);
  }

  // triangles
  const size_t ntris = msg.navcels_v0.size();
  nm.navcels.resize(ntris);
  for (size_t i = 0; i < ntris; ++i) {
    nm.navcels[i].v[0] = msg.navcels_v0[i];
    nm.navcels[i].v[1] = msg.navcels_v1[i];
    nm.navcels[i].v[2] = msg.navcels_v2[i];
  }

  // surfaces
  nm.surfaces.resize(msg.surfaces.size());
  for (size_t i = 0; i < msg.surfaces.size(); ++i) {
    nm.surfaces[i].frame_id = msg.surfaces[i].frame_id;
    nm.surfaces[i].navcels.assign(msg.surfaces[i].navcels.begin(),
                                  msg.surfaces[i].navcels.end());
  }

  // Fallback: if no surfaces were provided but there are triangles,
  // create a default single surface listing all triangles.
  if (nm.surfaces.empty() && ntris > 0) {
    navmap::Surface s;
    s.navcels.resize(ntris);
    for (navmap::NavCelId cid = 0; cid < static_cast<navmap::NavCelId>(ntris); ++cid) {
      s.navcels[cid] = cid;
    }
    nm.surfaces.push_back(std::move(s));
  }

  // layers
  for (const auto & l : msg.layers) {
    switch (l.type) {
      case 0: {
          auto v = nm.layers.add_or_get<uint8_t>(l.name, ntris, navmap::LayerType::U8);
          v->data() = l.data_u8;
        } break;
      case 1: {
          auto v = nm.layers.add_or_get<float>(l.name, ntris, navmap::LayerType::F32);
          v->data() = l.data_f32;
        } break;
      case 2: {
          auto v = nm.layers.add_or_get<double>(l.name, ntris, navmap::LayerType::F64);
          v->data() = l.data_f64;
        } break;
    }
  }

  // Derived
  nm.rebuild_geometry_accels();
  return nm;
}

navmap_ros_interfaces::msg::NavMapLayer to_msg(
  const navmap::NavMap & nm,
  const std::string & layer_name)
{
  navmap_ros_interfaces::msg::NavMapLayer msg;
  msg.name = layer_name;

  auto base = nm.layers.get(layer_name);
  if (!base) {
    throw std::runtime_error("to_msg(NavMapLayer): layer '" + layer_name + "' not found");
  }

  switch (base->type()) {
    case navmap::LayerType::U8: {
        msg.type = navmap_ros_interfaces::msg::NavMapLayer::U8;
        auto v = std::dynamic_pointer_cast<navmap::LayerView<uint8_t>>(base);
        msg.data_u8 = v->data();
        break;
      }
    case navmap::LayerType::F32: {
        msg.type = navmap_ros_interfaces::msg::NavMapLayer::F32;
        auto v = std::dynamic_pointer_cast<navmap::LayerView<float>>(base);
        msg.data_f32 = v->data();
        break;
      }
    case navmap::LayerType::F64: {
        msg.type = navmap_ros_interfaces::msg::NavMapLayer::F64;
        auto v = std::dynamic_pointer_cast<navmap::LayerView<double>>(base);
        msg.data_f64 = v->data();
        break;
      }
    default:
      throw std::runtime_error("to_msg(NavMapLayer): unsupported layer type");
  }

  return msg;
}

void from_msg(
  const navmap_ros_interfaces::msg::NavMapLayer & msg,
  navmap::NavMap & nm)
{
  switch (msg.type) {
    case navmap_ros_interfaces::msg::NavMapLayer::U8: {
        auto dst = nm.add_layer<uint8_t>(msg.name, /*desc*/"", /*unit*/"", uint8_t{});
        if (dst->data().size() != msg.data_u8.size()) {
          dst->data().resize(msg.data_u8.size());
        }
        std::copy(msg.data_u8.begin(), msg.data_u8.end(), dst->data().begin());
        break;
      }
    case navmap_ros_interfaces::msg::NavMapLayer::F32: {
        auto dst = nm.add_layer<float>(msg.name, /*desc*/"", /*unit*/"", 0.0f);
        if (dst->data().size() != msg.data_f32.size()) {
          dst->data().resize(msg.data_f32.size());
        }
        std::copy(msg.data_f32.begin(), msg.data_f32.end(), dst->data().begin());
        break;
      }
    case navmap_ros_interfaces::msg::NavMapLayer::F64: {
        auto dst = nm.add_layer<double>(msg.name, /*desc*/"", /*unit*/"", 0.0);
        if (dst->data().size() != msg.data_f64.size()) {
          dst->data().resize(msg.data_f64.size());
        }
        std::copy(msg.data_f64.begin(), msg.data_f64.end(), dst->data().begin());
        break;
      }
    default:
      throw std::runtime_error("from_msg(NavMapLayer): unsupported type value " +
        std::to_string(msg.type));
  }

  // No description/unit in the .msg schema; leave metadata empty or keep previous.
}

// ----------------- OccupancyGrid <-> NavMap -----------------

navmap::NavMap from_occupancy_grid(const nav_msgs::msg::OccupancyGrid & grid)
{
  navmap::NavMap nm;

  const uint32_t W = grid.info.width;
  const uint32_t H = grid.info.height;
  const float res = grid.info.resolution;
  const auto & O = grid.info.origin;

  // 1) Positions: shared vertices (W+1)*(H+1)
  nm.positions.x.reserve((W + 1) * (H + 1));
  nm.positions.y.reserve((W + 1) * (H + 1));
  nm.positions.z.reserve((W + 1) * (H + 1));
  for (uint32_t j = 0; j <= H; ++j) {
    for (uint32_t i = 0; i <= W; ++i) {
      nm.positions.x.push_back(static_cast<float>(O.position.x + i * res));
      nm.positions.y.push_back(static_cast<float>(O.position.y + j * res));
      nm.positions.z.push_back(static_cast<float>(O.position.z));
    }
  }
  auto v_id = [W](uint32_t i, uint32_t j) -> navmap::PointId {
      return static_cast<navmap::PointId>(j * (W + 1) + i);
    };

  // 2) Triangles: 2 per cell, diagonal pattern = 0
  nm.navcels.resize(static_cast<size_t>(2ull * W * H));
  size_t tidx = 0;
  for (uint32_t j = 0; j < H; ++j) {
    for (uint32_t i = 0; i < W; ++i) {
      // tri0: (i,j)-(i+1,j)-(i+1,j+1)
      {
        auto & c = nm.navcels[tidx++];
        c.v[0] = v_id(i, j);
        c.v[1] = v_id(i + 1, j);
        c.v[2] = v_id(i + 1, j + 1);
      }
      // tri1: (i,j)-(i+1,j+1)-(i,j+1)
      {
        auto & c = nm.navcels[tidx++];
        c.v[0] = v_id(i, j);
        c.v[1] = v_id(i + 1, j + 1);
        c.v[2] = v_id(i, j + 1);
      }
    }
  }

  // 3) One surface listing all triangles
  nm.surfaces.resize(1);
  nm.surfaces[0].frame_id = grid.header.frame_id;
  nm.surfaces[0].navcels.resize(nm.navcels.size());
  for (navmap::NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
    nm.surfaces[0].navcels[cid] = cid;
  }

  // 4) Derived geometry
  nm.rebuild_geometry_accels();

  // 5) Per-NavCel "occupancy" layer (U8), two triangles per cell with the same value
  auto occ = nm.layers.add_or_get<uint8_t>("occupancy", nm.navcels.size(), navmap::LayerType::U8);
  tidx = 0;
  auto cell_id = [W](uint32_t i, uint32_t j) {return j * W + i;};
  for (uint32_t j = 0; j < H; ++j) {
    for (uint32_t i = 0; i < W; ++i) {
      const int8_t src = grid.data[cell_id(i, j)];
      const uint8_t u8 = occ_to_u8(src);
      (*occ)[tidx + 0] = u8;
      (*occ)[tidx + 1] = u8;
      tidx += 2;
    }
  }

  return nm;
}

nav_msgs::msg::OccupancyGrid to_occupancy_grid(const navmap::NavMap & nm)
{
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = (nm.surfaces.empty() ? std::string() : nm.surfaces[0].frame_id);

  // Find occupancy layer
  auto base = nm.layers.get("occupancy");
  if (!base || base->type() != navmap::LayerType::U8) {
    // Fallback to empty grid
    g.info.width = 0;
    g.info.height = 0;
    g.info.resolution = 1.0f;
    g.data.clear();
    return g;
  }
  auto occ = std::dynamic_pointer_cast<navmap::LayerView<uint8_t>>(base);

  // Fast path: detect regular grid from positions and triangle layout.
  // Assumptions: single flat Z plane, regular spacing in X/Y, raster triangle order.
  if (nm.surfaces.size() == 1) {
    std::vector<float> xs(nm.positions.x.begin(), nm.positions.x.end());
    std::vector<float> ys(nm.positions.y.begin(), nm.positions.y.end());
    std::sort(xs.begin(), xs.end()); xs.erase(std::unique(xs.begin(), xs.end()), xs.end());
    std::sort(ys.begin(), ys.end()); ys.erase(std::unique(ys.begin(), ys.end()), ys.end());
    if (xs.size() >= 2 && ys.size() >= 2) {
      const float resx = xs[1] - xs[0];
      const float resy = ys[1] - ys[0];
      const float res = static_cast<float>(0.5 * (resx + resy));

      const uint32_t W = static_cast<uint32_t>(xs.size() - 1);
      const uint32_t H = static_cast<uint32_t>(ys.size() - 1);
      if (occ->size() == static_cast<size_t>(2ull * W * H)) {
        // Fill grid directly
        g.info.width = W;
        g.info.height = H;
        g.info.resolution = res;
        g.info.origin.position.x = xs.front();
        g.info.origin.position.y = ys.front();
        g.info.origin.position.z = nm.positions.z.empty() ? 0.0 : nm.positions.z.front();
        g.info.origin.orientation.w = 1.0;

        g.data.assign(static_cast<size_t>(W * H), 0);
        size_t tidx = 0;
        auto idx_cell = [W](uint32_t i, uint32_t j) {return j * W + i;};
        for (uint32_t j = 0; j < H; ++j) {
          for (uint32_t i = 0; i < W; ++i) {
            // Both triangle values should be equal; use the first.
            const uint8_t u8 = (*occ)[tidx];
            g.data[idx_cell(i, j)] = u8_to_occ(u8);
            tidx += 2;
          }
        }
        return g;
      }
    }
  }

  // Generic fallback: sample each cell center with closest_navcel()
  std::vector<float> xs(nm.positions.x.begin(), nm.positions.x.end());
  std::vector<float> ys(nm.positions.y.begin(), nm.positions.y.end());
  if (xs.size() < 2 || ys.size() < 2) {
    g.info.width = 0; g.info.height = 0; g.data.clear();
    return g;
  }
  std::sort(xs.begin(), xs.end()); xs.erase(std::unique(xs.begin(), xs.end()), xs.end());
  std::sort(ys.begin(), ys.end()); ys.erase(std::unique(ys.begin(), ys.end()), ys.end());
  const float res = static_cast<float>(0.5 * ((xs[1] - xs[0]) + (ys[1] - ys[0])));

  const uint32_t W = static_cast<uint32_t>(xs.size() - 1);
  const uint32_t H = static_cast<uint32_t>(ys.size() - 1);

  g.info.width = W;
  g.info.height = H;
  g.info.resolution = res;
  g.info.origin.position.x = xs.front();
  g.info.origin.position.y = ys.front();
  g.info.origin.position.z = nm.positions.z.empty() ? 0.0 : nm.positions.z.front();
  g.info.origin.orientation.w = 1.0;

  g.data.assign(static_cast<size_t>(W * H), -1);
  auto idx_cell = [W](uint32_t i, uint32_t j) {return j * W + i;};

  for (uint32_t j = 0; j < H; ++j) {
    for (uint32_t i = 0; i < W; ++i) {
      const float cx = g.info.origin.position.x + (i + 0.5f) * res;
      const float cy = g.info.origin.position.y + (j + 0.5f) * res;

      size_t sidx = 0;
      navmap::NavCelId cid = 0;
      Eigen::Vector3f closest;
      float sq = 0.0f;

      // Use closest_navcel (const) instead of locate_navcel (non-const)
      if (nm.closest_navcel({cx, cy, static_cast<float>(g.info.origin.position.z)},
                              sidx, cid, closest, sq))
      {
        const uint8_t u8 = (*occ)[cid];
        g.data[idx_cell(i, j)] = u8_to_occ(u8);
      } else {
        g.data[idx_cell(i, j)] = -1;
      }
    }
  }
  return g;
}

bool build_navmap_from_mesh(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const std::vector<Eigen::Vector3i> & triangles,
  const std::string & frame_id,
  navmap_ros_interfaces::msg::NavMap & out_msg,
  navmap::NavMap * out_core_opt)
{
  out_msg = navmap_ros_interfaces::msg::NavMap();
  out_msg.header.frame_id = frame_id;

  // 1) Vertices
  out_msg.positions_x.reserve(cloud.size());
  out_msg.positions_y.reserve(cloud.size());
  out_msg.positions_z.reserve(cloud.size());
  for (const auto & p : cloud) {
    out_msg.positions_x.push_back(p.x);
    out_msg.positions_y.push_back(p.y);
    out_msg.positions_z.push_back(p.z);
  }

  // 2) Triangles (validate indices)
  const std::size_t N = cloud.size();
  out_msg.navcels_v0.reserve(triangles.size());
  out_msg.navcels_v1.reserve(triangles.size());
  out_msg.navcels_v2.reserve(triangles.size());

  for (const auto & t : triangles) {
    const int i0 = t[0], i1 = t[1], i2 = t[2];
    if (i0 < 0 || i1 < 0 || i2 < 0 ||
      static_cast<std::size_t>(i0) >= N ||
      static_cast<std::size_t>(i1) >= N ||
      static_cast<std::size_t>(i2) >= N)
    {
      return false; // invalid index
    }
    out_msg.navcels_v0.push_back(static_cast<uint32_t>(i0));
    out_msg.navcels_v1.push_back(static_cast<uint32_t>(i1));
    out_msg.navcels_v2.push_back(static_cast<uint32_t>(i2));
  }

  // 3) One surface listing all triangles (required for raycasting/BVH)
  {
    navmap_ros_interfaces::msg::NavMapSurface s;
    s.frame_id = frame_id;
    const std::size_t ntris = out_msg.navcels_v0.size();
    s.navcels.resize(ntris);
    for (std::size_t cid = 0; cid < ntris; ++cid) {
      s.navcels[cid] = static_cast<uint32_t>(cid);
    }
    out_msg.surfaces.push_back(std::move(s));
  }

  // 4) Per-navcel "elevation" layer (float32): mean Z of triangle vertices
  {
    std::vector<float> elev;
    elev.reserve(triangles.size());
    for (const auto & t : triangles) {
      const int i0 = t[0], i1 = t[1], i2 = t[2];
      const float z0 = cloud[i0].z;
      const float z1 = cloud[i1].z;
      const float z2 = cloud[i2].z;
      const float mean_z = (z0 + z1 + z2) / 3.0f;
      elev.push_back(mean_z);
    }

    navmap_ros_interfaces::msg::NavMapLayer layer;
    layer.name = "elevation";
    layer.type = 1;                 // 0=u8, 1=f32, 2=f64
    layer.data_f32 = std::move(elev);
    out_msg.layers.push_back(std::move(layer));
  }

  // 5) Convert to core structure if requested
  if (out_core_opt) {
    *out_core_opt = navmap_ros::from_msg(out_msg);
  }
  return true;
}

// ----------------- Surface from PC2 -----------------

using Triangle = Eigen::Vector3i;

static inline float sqr(float v) {return v * v;}
static inline float dist3(const pcl::PointXYZ & a, const pcl::PointXYZ & b)
{
  return std::sqrt(sqr(a.x - b.x) + sqr(a.y - b.y) + sqr(a.z - b.z));
}

static inline float clamp01(float x)
{
  if (x < 0.0f) {return 0.0f;}
  if (x > 1.0f) {return 1.0f;}
  return x;
}

static inline float tri_area(
  const Eigen::Vector3f & a,
  const Eigen::Vector3f & b,
  const Eigen::Vector3f & c)
{
  return 0.5f * ((b - a).cross(c - a)).norm();
}

// Keys to avoid duplicates (edges/triangles)
struct EdgeKey
{
  int a, b;
  bool operator==(const EdgeKey & o) const noexcept {return a == o.a && b == o.b;}
};
struct EdgeHasher
{
  std::size_t operator()(const EdgeKey & e) const noexcept
  {
    return (static_cast<std::size_t>(e.a) << 32) ^ static_cast<std::size_t>(e.b);
  }
};
static inline EdgeKey make_edge(int i, int j)
{
  if (i < j) {return {i, j};}
  return {j, i};
}

struct TriKey
{
  int a, b, c;
  bool operator==(const TriKey & o) const noexcept
  {
    return a == o.a && b == o.b && c == o.c;
  }
};
struct TriHasher
{
  std::size_t operator()(const TriKey & t) const noexcept
  {
    std::size_t h = 1469598103934665603ull;
    auto mix = [&](int k){
        h ^= static_cast<std::size_t>(k) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
      };
    mix(t.a); mix(t.b); mix(t.c);
    return h;
  }
};
static inline TriKey make_tri(int i, int j, int k)
{
  int v[3] = {i, j, k};
  std::sort(v, v + 3);
  return {v[0], v[1], v[2]};
}

inline void triangle_angles_deg(
  const Eigen::Vector3f & A,
  const Eigen::Vector3f & B,
  const Eigen::Vector3f & C,
  float & angA, float & angB, float & angC)
{
  // Sides opposite to vertices A, B, C
  float a = (B - C).norm();
  float b = (C - A).norm();
  float c = (A - B).norm();

  // Avoid divisions by zero
  const float eps = 1e-12f;
  a = std::max(a, eps);
  b = std::max(b, eps);
  c = std::max(c, eps);

  // Law of cosines with numeric clamping
  auto angle_from = [](float opp, float x, float y) -> float {
      float cosv = (x * x + y * y - opp * opp) / (2.0f * x * y);
      cosv = std::min(1.0f, std::max(-1.0f, cosv));
      return std::acos(cosv) * 180.0f / static_cast<float>(M_PI);
    };

  angA = angle_from(a, b, c);
  angB = angle_from(b, c, a);
  angC = angle_from(c, a, b);
}

// ----------------- Downsampling -----------------

struct Voxel
{
  int x, y, z;
  bool operator==(const Voxel & o) const noexcept
  {
    return x == o.x && y == o.y && z == o.z;
  }
};

struct VoxelHash
{
  std::size_t operator()(const Voxel & v) const noexcept
  {
    // Simple, stable hash
    std::size_t h = 1469598103934665603ull; // FNV-1a offset
    auto mix = [&](int k) {
        std::size_t x = static_cast<std::size_t>(k);
        h ^= x + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
      };
    mix(v.x); mix(v.y); mix(v.z);
    return h;
  }
};

struct VoxelAccum
{
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_z = 0.0f;
  int count = 0;
};

// Compute hash grid cell index for a point
static inline Voxel cell_of_point(const pcl::PointXYZ & p, float cell)
{
  return Voxel{
    static_cast<int>(std::floor(p.x / cell)),
    static_cast<int>(std::floor(p.y / cell)),
    static_cast<int>(std::floor(p.z / cell))
  };
}

static inline pcl::PointXYZ accum_centroid(const VoxelAccum & a)
{
  const float inv = a.count > 0 ? 1.0f / static_cast<float>(a.count) : 0.0f;
  return pcl::PointXYZ(a.sum_x * inv, a.sum_y * inv, a.sum_z * inv);
}

// Downsampling by voxelization with two passes (normal and shifted grid),
// followed by merging centroids that fall within 0.5*resolution
pcl::PointCloud<pcl::PointXYZ>
downsample_voxelize_avgXYZ(
  const pcl::PointCloud<pcl::PointXYZ> & input_points,
  float resolution)
{
  pcl::PointCloud<pcl::PointXYZ> output;

  if (input_points.empty() || !(resolution > 0.0f)) {
    output.width = 0; output.height = 1; output.is_dense = true;
    return output;
  }

  const float r = resolution;
  const float half = 0.5f * r;

  // Pass A: regular grid
  std::unordered_map<Voxel, VoxelAccum, VoxelHash> mapA;
  mapA.reserve(input_points.size() / 2 + 1);

  for (const auto & pt : input_points.points) {
    if (!pcl::isFinite(pt)) {continue;}
    Voxel v{
      static_cast<int>(std::floor(pt.x / r)),
      static_cast<int>(std::floor(pt.y / r)),
      static_cast<int>(std::floor(pt.z / r))
    };
    auto & acc = mapA[v];
    acc.sum_x += pt.x; acc.sum_y += pt.y; acc.sum_z += pt.z; acc.count += 1;
  }

  std::vector<pcl::PointXYZ> centroids_a;
  centroids_a.reserve(mapA.size());
  for (const auto & kv : mapA) {
    centroids_a.push_back(accum_centroid(kv.second));
  }

  // Pass B: grid shifted by half resolution
  std::unordered_map<Voxel, VoxelAccum, VoxelHash> mapB;
  mapB.reserve(input_points.size() / 2 + 1);
  for (const auto & pt : input_points.points) {
    if (!pcl::isFinite(pt)) {continue;}
    Voxel v{
      static_cast<int>(std::floor((pt.x - half) / r)),
      static_cast<int>(std::floor((pt.y - half) / r)),
      static_cast<int>(std::floor((pt.z - half) / r))
    };
    auto & acc = mapB[v];
    acc.sum_x += pt.x; acc.sum_y += pt.y; acc.sum_z += pt.z; acc.count += 1;
  }

  std::vector<pcl::PointXYZ> centroids_b;
  centroids_b.reserve(mapB.size());
  for (const auto & kv : mapB) {
    centroids_b.push_back(accum_centroid(kv.second));
  }

  // Merge centroids using a hash grid to join those split by voxel borders
  const float merge_radius = half;
  const float merge_radius2 = merge_radius * merge_radius;
  const float grid_cell_size = r;

  std::vector<VoxelAccum> clusters;
  clusters.reserve(centroids_a.size());

  std::unordered_map<Voxel, std::vector<int>, VoxelHash> grid;
  grid.reserve(centroids_a.size() + centroids_b.size());

  auto try_insert = [&](const pcl::PointXYZ & p)
    {
      Voxel c = cell_of_point(p, grid_cell_size);
      int best_idx = -1;
      float best_d2 = std::numeric_limits<float>::max();

      // Search 27 neighboring cells for a close cluster
      for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            Voxel nb{c.x + dx, c.y + dy, c.z + dz};
            auto it = grid.find(nb);
            if (it == grid.end()) {continue;}

            for (int idx : it->second) {
              const pcl::PointXYZ q = accum_centroid(clusters[idx]);
              const float ex = p.x - q.x;
              const float ey = p.y - q.y;
              const float ez = p.z - q.z;
              const float d2 = ex * ex + ey * ey + ez * ez;
              if (d2 < best_d2) {best_d2 = d2; best_idx = idx;}
            }
          }
        }
      }

      if (best_idx >= 0 && best_d2 <= merge_radius2) {
        // Merge into existing cluster
        clusters[best_idx].sum_x += p.x;
        clusters[best_idx].sum_y += p.y;
        clusters[best_idx].sum_z += p.z;
        clusters[best_idx].count += 1;
      } else {
        // Create new cluster
        VoxelAccum acc;
        acc.sum_x = p.x; acc.sum_y = p.y; acc.sum_z = p.z; acc.count = 1;
        int new_idx = static_cast<int>(clusters.size());
        clusters.push_back(acc);
        grid[c].push_back(new_idx);
      }
    };

  // Insert centroids from both passes
  for (const auto & p : centroids_a) {
    try_insert(p);
  }
  for (const auto & p : centroids_b) {
    try_insert(p);
  }

  // Emit one point per cluster
  output.points.reserve(clusters.size());
  for (const auto & cl : clusters) {
    output.points.push_back(accum_centroid(cl));
  }

  output.width = static_cast<uint32_t>(output.points.size());
  output.height = 1;
  output.is_dense = true;
  return output;
}

// Downsampling que NO colapsa plantas: un punto por voxel XY con Z media
pcl::PointCloud<pcl::PointXYZ>
downsample_voxelize_avgZ(
  const pcl::PointCloud<pcl::PointXYZ> & input_points,
  float resolution)
{
  pcl::PointCloud<pcl::PointXYZ> output;

  if (input_points.empty() || !(resolution > 0.0f)) {
    output.width = 0; output.height = 1; output.is_dense = true;
    return output;
  }

  struct Accum
  {
    double sum_z{0.0};
    int count{0};
  };

  std::unordered_map<Voxel, Accum, VoxelHash> voxels;
  voxels.reserve(input_points.size() / 2);

  // Accumulate Z per voxel
  for (const auto & pt : input_points) {
    if (!pcl::isFinite(pt)) {continue;}

    int vx = static_cast<int>(std::floor(pt.x / resolution));
    int vy = static_cast<int>(std::floor(pt.y / resolution));
    int vz = static_cast<int>(std::floor(pt.z / resolution)); // only for voxel id

    Voxel v{vx, vy, vz};
    auto & acc = voxels[v];
    acc.sum_z += pt.z;
    acc.count += 1;
  }

  // Emit one point per voxel (centro XY del voxel y Z media)
  output.points.reserve(voxels.size());
  for (const auto & kv : voxels) {
    const Voxel & v = kv.first;
    const Accum & acc = kv.second;

    // Center of voxel in XY
    float cx = (v.x + 0.5f) * resolution;
    float cy = (v.y + 0.5f) * resolution;
    // Average Z of all points
    float cz = static_cast<float>(acc.sum_z / acc.count);

    output.emplace_back(cx, cy, cz);
  }

  output.width = static_cast<uint32_t>(output.points.size());
  output.height = 1;
  output.is_dense = true;

  return output;
}

// Downsampling “Top-Z”: un punto por voxel XY con el mayor Z observado (evita contaminar rampas)
pcl::PointCloud<pcl::PointXYZ>
downsample_voxelize_topZ(
  const pcl::PointCloud<pcl::PointXYZ> & input_points,
  float resolution)
{
  pcl::PointCloud<pcl::PointXYZ> output;
  if (input_points.empty() || !(resolution > 0.0f)) {
    output.width = 0; output.height = 1; output.is_dense = true;
    return output;
  }

  struct Accum {
    float cx, cy, zmax;
    bool has{false};
  };

  std::unordered_map<Voxel, Accum, VoxelHash> vox;
  vox.reserve(input_points.size() / 2);

  for (const auto & pt : input_points) {
    if (!pcl::isFinite(pt)) continue;

    const int vx = static_cast<int>(std::floor(pt.x / resolution));
    const int vy = static_cast<int>(std::floor(pt.y / resolution));
    const int vz = static_cast<int>(std::floor(pt.z / resolution)); // solo para ID
    Voxel v{vx, vy, vz};

    auto & a = vox[v];
    if (!a.has) {
      a.has  = true;
      a.cx   = (vx + 0.5f) * resolution;   // centro XY del voxel
      a.cy   = (vy + 0.5f) * resolution;
      a.zmax = pt.z;
    } else if (pt.z > a.zmax) {
      a.zmax = pt.z;
    }
  }

  output.points.reserve(vox.size());
  for (const auto & kv : vox) {
    const auto & a = kv.second;
    output.emplace_back(a.cx, a.cy, a.zmax);
  }

  output.width = static_cast<uint32_t>(output.points.size());
  output.height = 1;
  output.is_dense = true;
  return output;
}

// === Downsample con capas por Z (un punto por grupo en cada celda XY) ===
// Si params.max_dz <= 0, usamos un fallback razonable (p.ej. 0.3 m).

pcl::PointCloud<pcl::PointXYZ>
downsample_voxelize_avgZ_layered(
  const pcl::PointCloud<pcl::PointXYZ> & input_points,
  float resolution,
  float dz_merge /* usa params.max_dz */)
{
  pcl::PointCloud<pcl::PointXYZ> output;
  if (input_points.empty() || !(resolution > 0.0f)) {
    output.width = 0; output.height = 1; output.is_dense = true;
    return output;
  }
  if (!(dz_merge > 0.0f)) dz_merge = 0.30f; // fallback

  struct VoxelXY { int x, y; };
  struct HashXY {
    size_t operator()(const VoxelXY& v) const noexcept {
      return (static_cast<size_t>(v.x) * 73856093u) ^ (static_cast<size_t>(v.y) * 19349663u);
    }
  };
  struct EqXY {
    bool operator()(const VoxelXY& a, const VoxelXY& b) const noexcept {
      return a.x == b.x && a.y == b.y;
    }
  };

  // Recolecta puntos por celda XY
  std::unordered_map<VoxelXY, std::vector<int>, HashXY, EqXY> buckets;
  buckets.reserve(input_points.size() / 4);

  for (int i = 0; i < static_cast<int>(input_points.size()); ++i) {
    const auto &pt = input_points[i];
    if (!pcl::isFinite(pt)) continue;
    const int vx = static_cast<int>(std::floor(pt.x / resolution));
    const int vy = static_cast<int>(std::floor(pt.y / resolution));
    buckets[{vx, vy}].push_back(i);
  }

  output.points.reserve(buckets.size()); // mínimo

  // Por cada celda, clusterea por Z y emite 1 punto por cluster (X,Y,Z = medias)
  for (auto & kv : buckets) {
    auto & idxs = kv.second;
    if (idxs.empty()) continue;

    std::sort(idxs.begin(), idxs.end(),
              [&](int a, int b){ return input_points[a].z < input_points[b].z; });

    // Acumuladores del cluster actual
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    int    count = 0;
    float  last_z = input_points[idxs.front()].z;

    auto flush_cluster = [&](){
      if (count <= 0) return;
      const float cx = static_cast<float>(sum_x / count);
      const float cy = static_cast<float>(sum_y / count);
      const float cz = static_cast<float>(sum_z / count);
      output.emplace_back(cx, cy, cz);
      sum_x = sum_y = sum_z = 0.0; count = 0;
    };

    for (int ii = 0; ii < static_cast<int>(idxs.size()); ++ii) {
      const auto &p = input_points[idxs[ii]];
      if (count == 0) {
        // inicia cluster
        sum_x = p.x; sum_y = p.y; sum_z = p.z; count = 1; last_z = p.z;
      } else {
        const float dz = std::fabs(p.z - last_z);
        if (dz <= dz_merge) {
          sum_x += p.x; sum_y += p.y; sum_z += p.z; ++count; last_z = p.z;
        } else {
          // salta a otra capa
          flush_cluster();
          sum_x = p.x; sum_y = p.y; sum_z = p.z; count = 1; last_z = p.z;
        }
      }
    }
    flush_cluster();
  }

  output.width = static_cast<uint32_t>(output.points.size());
  output.height = 1;
  output.is_dense = true;
  return output;
}

pcl::PointCloud<pcl::PointXYZ>
downsample_voxelize_topZ_layered(
  const pcl::PointCloud<pcl::PointXYZ> & input_points,
  float resolution,
  float dz_merge /* usa params.max_dz */)
{
  pcl::PointCloud<pcl::PointXYZ> output;
  if (input_points.empty() || !(resolution > 0.0f)) {
    output.width = 0; output.height = 1; output.is_dense = true;
    return output;
  }
  if (!(dz_merge > 0.0f)) dz_merge = 0.30f; // fallback

  struct VoxelXY { int x, y; };
  struct HashXY {
    size_t operator()(const VoxelXY& v) const noexcept {
      return (static_cast<size_t>(v.x) * 73856093u) ^ (static_cast<size_t>(v.y) * 19349663u);
    }
  };
  struct EqXY {
    bool operator()(const VoxelXY& a, const VoxelXY& b) const noexcept {
      return a.x == b.x && a.y == b.y;
    }
  };

  std::unordered_map<VoxelXY, std::vector<int>, HashXY, EqXY> buckets;
  buckets.reserve(input_points.size() / 4);

  for (int i = 0; i < static_cast<int>(input_points.size()); ++i) {
    const auto &pt = input_points[i];
    if (!pcl::isFinite(pt)) continue;
    const int vx = static_cast<int>(std::floor(pt.x / resolution));
    const int vy = static_cast<int>(std::floor(pt.y / resolution));
    buckets[{vx, vy}].push_back(i);
  }

  output.points.reserve(buckets.size());

  for (auto & kv : buckets) {
    auto & idxs = kv.second;
    if (idxs.empty()) continue;

    std::sort(idxs.begin(), idxs.end(),
              [&](int a, int b){ return input_points[a].z < input_points[b].z; });

    // Cluster por Z; emitimos punto con X,Y medios y Z = z_max del cluster
    double sum_x = 0.0, sum_y = 0.0;
    float  z_max = -std::numeric_limits<float>::infinity();
    int    count = 0;
    float  last_z = input_points[idxs.front()].z;

    auto flush_cluster = [&](){
      if (count <= 0) return;
      const float cx = static_cast<float>(sum_x / count);
      const float cy = static_cast<float>(sum_y / count);
      output.emplace_back(cx, cy, z_max);
      sum_x = 0.0; sum_y = 0.0; z_max = -std::numeric_limits<float>::infinity(); count = 0;
    };

    for (int ii = 0; ii < static_cast<int>(idxs.size()); ++ii) {
      const auto &p = input_points[idxs[ii]];
      if (count == 0) {
        sum_x = p.x; sum_y = p.y; z_max = p.z; count = 1; last_z = p.z;
      } else {
        const float dz = std::fabs(p.z - last_z);
        if (dz <= dz_merge) {
          sum_x += p.x; sum_y += p.y; z_max = std::max(z_max, p.z); ++count; last_z = p.z;
        } else {
          flush_cluster();
          sum_x = p.x; sum_y = p.y; z_max = p.z; count = 1; last_z = p.z;
        }
      }
    }
    flush_cluster();
  }

  output.width = static_cast<uint32_t>(output.points.size());
  output.height = 1;
  output.is_dense = true;
  return output;
}


// ----------------- Attempt to add triangle (fast filters first) -----------------
// Incluye: max_edge_len, max_dz, neighbor_radius (XY), min_area, min_angle, pendiente por n·z
inline bool try_add_triangle(
  int i, int j, int k,
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const BuildParams & P,
  std::unordered_set<TriKey, TriHasher> & tri_set,
  std::unordered_set<EdgeKey, EdgeHasher> & edge_set,
  std::vector<Triangle> & tris)
{
  TriKey tk = make_tri(i, j, k);
  if (tri_set.find(tk) != tri_set.end()) {return false;}

  const auto & A = cloud[i];
  const auto & B = cloud[j];
  const auto & C = cloud[k];
  if (!pcl::isFinite(A) || !pcl::isFinite(B) || !pcl::isFinite(C)) {return false;}

  // Max edge length (3D)
  auto dAB = dist3(A, B);
  auto dBC = dist3(B, C);
  auto dCA = dist3(C, A);
  if (P.max_edge_len > 0.0f &&
      (dAB > P.max_edge_len || dBC > P.max_edge_len || dCA > P.max_edge_len)) {return false;}

  // Max |Δz| per edge
  if (P.max_dz > 0.0f) {
    const float dzAB = std::fabs(A.z - B.z);
    const float dzBC = std::fabs(B.z - C.z);
    const float dzCA = std::fabs(C.z - A.z);
    if (std::max({dzAB, dzBC, dzCA}) > P.max_dz) {return false;}
  }


  Eigen::Vector3f a(A.x, A.y, A.z);
  Eigen::Vector3f b(B.x, B.y, B.z);
  Eigen::Vector3f c(C.x, C.y, C.z);

  // Min area
  if (tri_area(a, b, c) < std::max(P.min_area, 1e-9f)) {return false;}

  // Max slope: usa n·z >= cos(max_slope) (sin acos)
  const float cos_max_slope =
      std::cos(P.max_slope_deg * static_cast<float>(M_PI) / 180.0f);
  Eigen::Vector3f n = (b - a).cross(c - a);
  const float nn = n.norm();
  if (nn < 1e-9f) {return false;}
  n /= nn;
  if (n.dot(Eigen::Vector3f::UnitZ()) < cos_max_slope) {return false;}

  // Min angle en cada vértice
  float angA, angB, angC;
  triangle_angles_deg(a, b, c, angA, angB, angC);
  if (angA < P.min_angle_deg || angB < P.min_angle_deg || angC < P.min_angle_deg) {
    return false;
  }

  // Orientación coherente (normal hacia +Z)
  if (n.dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
    std::swap(j, k);
  }

  // Accept
  tri_set.insert(tk);
  edge_set.insert(make_edge(i, j));
  edge_set.insert(make_edge(j, k));
  edge_set.insert(make_edge(k, i));
  tris.emplace_back(Triangle(i, j, k));
  return true;
}

// ----------------- Crecimiento local desde semilla (acelerado) -----------------

std::vector<Triangle> grow_surface_from_seed(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  int seed_idx,
  const BuildParams & P)
{
  std::vector<Triangle> tris;
  if (cloud.empty() || seed_idx < 0 || seed_idx >= static_cast<int>(cloud.size())) {
    return tris;
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud.makeShared());

  std::queue<int> frontier;
  std::unordered_set<int> seen;
  frontier.push(seed_idx);
  seen.insert(seed_idx);

  std::unordered_set<TriKey, TriHasher> tri_set;
  std::unordered_set<EdgeKey, EdgeHasher> edge_set;

  const int MAX_NEIGH_SEED = 32;  // cap vecinos semilla
  const int MAX_NEIGH_BFS  = 24;  // cap vecinos por arista

  auto dist2XY = [](const pcl::PointXYZ &P, const pcl::PointXYZ &Q){
    const float dx=P.x-Q.x, dy=P.y-Q.y; return dx*dx+dy*dy;
  };

  while (!frontier.empty()) {
    int v = frontier.front(); frontier.pop();
    const auto & V = cloud[v];
    if (!pcl::isFinite(V)) {continue;}

    // 1) Vecindad de la semilla (cap por densidad)
    std::vector<int> neigh; std::vector<float> dists;
    if (P.neighbor_radius > 0.0f) {
      kdtree.radiusSearch(V, P.neighbor_radius, neigh, dists);
      // ordenamos por d² en XY y cap
      std::sort(neigh.begin(), neigh.end(), [&](int a, int b){
        return dist2XY(cloud[v], cloud[a]) < dist2XY(cloud[v], cloud[b]);
      });
      if ((int)neigh.size() > MAX_NEIGH_SEED) neigh.resize(MAX_NEIGH_SEED);
    } else {
      const int K = std::max(P.k_neighbors, MAX_NEIGH_SEED);
      kdtree.nearestKSearch(V, K, neigh, dists);
    }

    // elimina self
    neigh.erase(std::remove(neigh.begin(), neigh.end(), v), neigh.end());
    if (neigh.size() < 2) continue;

    // 2) Orden angular simple en XY
    std::sort(neigh.begin(), neigh.end(), [&](int a, int b){
      const float ax = cloud[a].x - V.x, ay = cloud[a].y - V.y;
      const float bx = cloud[b].x - V.x, by = cloud[b].y - V.y;
      return std::atan2(ay, ax) < std::atan2(by, bx);
    });

    // 3) Fan
    for (size_t t=0; t+1<neigh.size(); ++t) {
      int j = neigh[t], k = neigh[t+1];
      if (try_add_triangle(v, j, k, cloud, P, tri_set, edge_set, tris)) {
        if (!seen.count(j)) {frontier.push(j); seen.insert(j);}
        if (!seen.count(k)) {frontier.push(k); seen.insert(k);}
      }
    }
    if (neigh.size() >= 3) {
      int j = neigh.back(), k = neigh.front();
      try_add_triangle(v, j, k, cloud, P, tri_set, edge_set, tris);
    }

    // 4) BFS por aristas recientes: ya se gestiona en caller en la variante con trazas
  }

  return tris;
}

// ----------------- Banded Delaunay (gap-based) -----------------

// Separa la nube en "bandas" siguiendo saltos en Z.
static std::vector<std::vector<int>>
split_points_by_z_gaps(const pcl::PointCloud<pcl::PointXYZ> & cloud,
                       float gap_z,
                       int   min_points_per_band = 3)
{
  std::vector<int> idx(cloud.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::sort(idx.begin(), idx.end(),
            [&](int a, int b){ return cloud[a].z < cloud[b].z; });

  std::vector<std::vector<int>> bands;
  if (idx.empty()) return bands;

  std::vector<int> cur;
  cur.reserve(256);
  cur.push_back(idx[0]);

  for (size_t t = 1; t < idx.size(); ++t) {
    const int i_prev = idx[t-1];
    const int i_curr = idx[t];
    const float dz = cloud[i_curr].z - cloud[i_prev].z;

    if (dz > gap_z) {
      if ((int)cur.size() >= min_points_per_band) bands.push_back(std::move(cur));
      cur.clear();
    }
    cur.push_back(i_curr);
  }
  if ((int)cur.size() >= min_points_per_band) bands.push_back(std::move(cur));

  return bands;
}



// ---- from_points: Delaunay por banda + filtros 3D (slope, max_dz, neighbor_radius, etc.)

navmap::NavMap from_points(
  const pcl::PointCloud<pcl::PointXYZ> & input_points,
  navmap_ros_interfaces::msg::NavMap & out_msg,
  BuildParams params)
{
  using std::vector;

  out_msg = navmap_ros_interfaces::msg::NavMap();
  if (input_points.empty() || input_points.size() < 3) {
    return navmap::NavMap();
  }

  // 1) Downsample que NO colapsa plantas (promedia Z por voxel XY)
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (params.resolution > 0.0f) {
    cloud = downsample_voxelize_avgZ_layered(input_points, params.resolution,
                                         (params.max_dz > 0.0f ? params.max_dz : 0.30f));
  } else {
    cloud = input_points;
  }
  if (cloud.size() < 3) {
    return navmap::NavMap();
  }

  // 2) Bandas por SALTOS en Z
  const float gap_z = (params.max_dz > 0.0f)
                        ? std::max(1.0f, 3.0f * params.max_dz)
                        : 1.0f;
  auto bands = split_points_by_z_gaps(cloud, gap_z, /*min_points_per_band=*/3);
  if (bands.empty()) {
    return navmap::NavMap();
  }

  // 3) Delaunay 2D (XY) por banda + filtros 3D
  struct Pt2 { double x, y; int idx3d; };
  struct Tri2 { int a, b, c; };

  auto orient2d = [](const Pt2 & a, const Pt2 & b, const Pt2 & c) -> double {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  };
  auto in_circumcircle = [&](const Pt2 & p, const Pt2 & A, const Pt2 & B, const Pt2 & C) -> bool {
    const double ax = A.x - p.x, ay = A.y - p.y;
    const double bx = B.x - p.x, by = B.y - p.y;
    const double cx = C.x - p.x, cy = C.y - p.y;
    const double det = (ax*ax + ay*ay) * (bx*cy - by*cx)
                     - (bx*bx + by*by) * (ax*cy - ay*cx)
                     + (cx*cx + cy*cy) * (ax*by - ay*bx);
    const double o = orient2d(A, B, C);
    return (o > 0.0) ? (det > 0.0) : (det < 0.0);
  };

  vector<Eigen::Vector3i> all_triangles;
  all_triangles.reserve(static_cast<size_t>(cloud.size()) * 2);

  vector<std::pair<size_t, size_t>> band_tri_ranges; // [offset, count] por banda

  const float cos_max_slope =
      std::cos(params.max_slope_deg * static_cast<float>(M_PI) / 180.0f);
  const float min_area = std::max(params.min_area, 1e-9f);
  const float max_edge =
      (params.max_edge_len > 0.0f) ? params.max_edge_len
                                   : std::numeric_limits<float>::infinity();
  const float max_dz =
      (params.max_dz > 0.0f) ? params.max_dz
                             : std::numeric_limits<float>::infinity();
  const float neigh_xy =
      (params.neighbor_radius > 0.0f) ? params.neighbor_radius
                                      : std::numeric_limits<float>::infinity();

  auto edge_len3D = [&](int i, int j) -> float {
    const auto & A = cloud[i];
    const auto & B = cloud[j];
    const float dx = A.x - B.x, dy = A.y - B.y, dz = A.z - B.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  };
  auto distXY = [&](const pcl::PointXYZ & P, const pcl::PointXYZ & Q) -> float {
    const float dx = P.x - Q.x, dy = P.y - Q.y;
    return std::sqrt(dx*dx + dy*dy);
  };

  for (const auto & band : bands) {
    if (band.size() < 3) {
      band_tri_ranges.emplace_back(all_triangles.size(), 0);
      continue;
    }

    // Puntos 2D de la banda
    vector<Pt2> vs; vs.reserve(band.size() + 3);
    double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
    for (int i3d : band) {
      const auto & p = cloud[i3d];
      vs.push_back({static_cast<double>(p.x), static_cast<double>(p.y), i3d});
      minx = std::min(minx, (double)p.x);
      miny = std::min(miny, (double)p.y);
      maxx = std::max(maxx, (double)p.x);
      maxy = std::max(maxy, (double)p.y);
    }

    // Super-triángulo
    const double dx = maxx - minx, dy = maxy - miny, d = std::max(dx, dy);
    const int iS1 = (int)vs.size(); vs.push_back({minx - 10*d, miny - d,       -1});
    const int iS2 = (int)vs.size(); vs.push_back({minx + 0.5*d, maxy + 10*d,   -2});
    const int iS3 = (int)vs.size(); vs.push_back({maxx + 10*d,  miny - d,      -3});

    vector<Tri2> tris2; tris2.push_back({iS1, iS2, iS3});

    // Bowyer–Watson
    for (int pi = 0; pi < (int)band.size(); ++pi) {
      const Pt2 Pp = vs[pi];
      vector<int> bad; bad.reserve(tris2.size());
      for (int t = 0; t < (int)tris2.size(); ++t) {
        const auto & T = tris2[t];
        if (in_circumcircle(Pp, vs[T.a], vs[T.b], vs[T.c])) bad.push_back(t);
      }

      // borde de cavidad
      struct EdgeBW { int a, b; };
      vector<EdgeBW> poly; poly.reserve(bad.size() * 3);
      std::vector<char> removed(tris2.size(), 0);
      for (int bi : bad) removed[bi] = 1;

      auto add_edge = [&](int a, int b){
        for (auto it = poly.begin(); it != poly.end(); ++it) {
          if (it->a == b && it->b == a) { poly.erase(it); return; }
        }
        poly.push_back({a, b});
      };

      for (int t = 0; t < (int)tris2.size(); ++t) {
        if (!removed[t]) continue;
        const auto & T = tris2[t];
        add_edge(T.a, T.b);
        add_edge(T.b, T.c);
        add_edge(T.c, T.a);
      }

      vector<Tri2> keep; keep.reserve(tris2.size());
      for (int t = 0; t < (int)tris2.size(); ++t) {
        if (!removed[t]) keep.push_back(tris2[t]);
      }
      tris2.swap(keep);

      for (const auto & e : poly) tris2.push_back({e.a, e.b, pi});
    }

    // Triángulos finales (sin super-triángulo)
    vector<Tri2> final_tris2; final_tris2.reserve(tris2.size());
    for (const auto & T : tris2) {
      if (T.a >= iS1 && T.a <= iS3) continue;
      if (T.b >= iS1 && T.b <= iS3) continue;
      if (T.c >= iS1 && T.c <= iS3) continue;
      final_tris2.push_back(T);
    }

    // Filtros 3D
    const size_t tri_off = all_triangles.size();
    size_t accepted = 0;

    for (const auto & T : final_tris2) {
      const int ia = vs[T.a].idx3d;
      const int ib = vs[T.b].idx3d;
      const int ic = vs[T.c].idx3d;

      const Eigen::Vector3f A(cloud[ia].x, cloud[ia].y, cloud[ia].z);
      const Eigen::Vector3f B(cloud[ib].x, cloud[ib].y, cloud[ib].z);
      const Eigen::Vector3f C(cloud[ic].x, cloud[ic].y, cloud[ic].z);

      const float area = tri_area(A, B, C);
      if (area < min_area) continue;

      const float lAB = edge_len3D(ia, ib);
      const float lBC = edge_len3D(ib, ic);
      const float lCA = edge_len3D(ic, ia);
      if (lAB > max_edge || lBC > max_edge || lCA > max_edge) continue;

      const float dzAB = std::fabs(A.z() - B.z());
      const float dzBC = std::fabs(B.z() - C.z());
      const float dzCA = std::fabs(C.z() - A.z());
      if (std::max({dzAB, dzBC, dzCA}) > max_dz) continue;

      if (neigh_xy < std::numeric_limits<float>::infinity()) {
        const auto & A3 = cloud[ia];
        const auto & B3 = cloud[ib];
        const auto & C3 = cloud[ic];
        if (distXY(A3, B3) > neigh_xy ||
            distXY(B3, C3) > neigh_xy ||
            distXY(C3, A3) > neigh_xy) {
          continue;
        }
      }

      Eigen::Vector3f n = (B - A).cross(C - A);
      const float nn = n.norm();
      if (nn < 1e-6f) continue;
      n /= nn;
      if (n.dot(Eigen::Vector3f::UnitZ()) < cos_max_slope) continue;

      all_triangles.emplace_back(ia, ib, ic);
      ++accepted;
    }

    band_tri_ranges.emplace_back(tri_off, accepted);
  }

  if (all_triangles.empty()) {
    return navmap::NavMap();
  }

  // 4) Construcción del NavMap
  const std::string frame_id = "map";
  navmap::NavMap core;
  if (!build_navmap_from_mesh(cloud, all_triangles, frame_id, out_msg, &core)) {
    out_msg = navmap_ros_interfaces::msg::NavMap();
    return navmap::NavMap();
  }

  // Reclasifica surfaces por banda asumiendo que se preserva el orden.
  out_msg.surfaces.clear();
  for (const auto & oc : band_tri_ranges) {
    const size_t off = oc.first, cnt = oc.second;
    if (cnt == 0) continue;
    navmap_ros_interfaces::msg::NavMapSurface s;
    s.frame_id = frame_id;
    s.navcels.resize(cnt);
    for (size_t k = 0; k < cnt; ++k) {
      s.navcels[k] = static_cast<uint32_t>(off + k);
    }
    out_msg.surfaces.push_back(std::move(s));
  }

  navmap::NavMap nm = from_msg(out_msg);
  return nm;
}

// Mantiene solo las 'max_surfaces' superficies más grandes (por nº de navcels), en orden descendente.
// max_surfaces <= 0 => no hace nada.
static void keep_top_surfaces_by_size(navmap_ros_interfaces::msg::NavMap & msg, int max_surfaces)
{
  if (max_surfaces <= 0) return;
  if (msg.surfaces.size() <= static_cast<size_t>(max_surfaces)) return;

  // Índices 0..S-1
  std::vector<size_t> ids(msg.surfaces.size());
  std::iota(ids.begin(), ids.end(), 0);

  // Ordenar por tamaño de superficie (desc)
  std::sort(ids.begin(), ids.end(), [&](size_t a, size_t b){
    return msg.surfaces[a].navcels.size() > msg.surfaces[b].navcels.size();
  });

  // Construir nueva lista con las N primeras
  std::vector<navmap_ros_interfaces::msg::NavMapSurface> kept;
  kept.reserve(static_cast<size_t>(max_surfaces));
  for (int i = 0; i < max_surfaces; ++i) {
    kept.push_back(std::move(msg.surfaces[ids[static_cast<size_t>(i)]]));
  }
  msg.surfaces.swap(kept);
}


// --- Reconstrucción de surfaces por conectividad de triángulos ---
// Agrupa triángulos en componentes conectadas por ARISTAS compartidas.
// No aplica ningún límite: conserva todas las superficies, ordenadas por tamaño (desc).
// --- Reconstrucción de surfaces por conectividad (rápida, limitada a triángulos ya conservados) ---
static void rebuild_surfaces_by_connectivity(navmap_ros_interfaces::msg::NavMap & msg)
{
  const size_t ntris = msg.navcels_v0.size();
  if (ntris == 0) { msg.surfaces.clear(); return; }

  // 1) Recolecta el conjunto de triángulos que realmente queremos considerar (los que están en surfaces actuales).
  //    Si no hay surfaces (caso raro), considera todos.
  std::vector<char> keep(ntris, 0);
  size_t kept_count = 0;
  if (!msg.surfaces.empty()) {
    for (const auto & s : msg.surfaces) {
      for (uint32_t cid : s.navcels) {
        if (cid < ntris && !keep[cid]) { keep[cid] = 1; ++kept_count; }
      }
    }
  } else {
    // Fallback: sin filtro
    kept_count = ntris;
    std::fill(keep.begin(), keep.end(), 1);
  }
  if (kept_count == 0) { msg.surfaces.clear(); return; }

  // 2) Mapea tri_global -> tri_local (0..K-1) y lista inversa.
  std::vector<uint32_t> glob2loc(ntris, UINT32_MAX);
  std::vector<uint32_t> loc2glob;
  loc2glob.reserve(kept_count);
  for (uint32_t t = 0; t < ntris; ++t) {
    if (keep[t]) {
      glob2loc[t] = static_cast<uint32_t>(loc2glob.size());
      loc2glob.push_back(t);
    }
  }
  const uint32_t K = static_cast<uint32_t>(loc2glob.size());

  // 3) Estructura Union-Find (DSU)
  std::vector<uint32_t> parent(K), rankv(K, 0);
  std::iota(parent.begin(), parent.end(), 0);
  auto findp = [&](uint32_t x) {
    while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
    return x;
  };
  auto unite = [&](uint32_t a, uint32_t b) {
    a = findp(a); b = findp(b);
    if (a == b) return;
    if (rankv[a] < rankv[b]) std::swap(a, b);
    parent[b] = a;
    if (rankv[a] == rankv[b]) ++rankv[a];
  };

  // 4) Mapa de arista -> tri_local (usa clave 64-bit con (min(v), max(v)))
  struct U64Hash {
    size_t operator()(const uint64_t &k) const noexcept { return static_cast<size_t>(k ^ (k >> 33)); }
  };
  auto edge_key = [](uint32_t a, uint32_t b) -> uint64_t {
    const uint64_t mn = (a < b) ? a : b;
    const uint64_t mx = (a < b) ? b : a;
    return (mn << 32) | mx;
  };

  std::unordered_map<uint64_t, uint32_t, U64Hash> edge2tri;
  edge2tri.reserve(static_cast<size_t>(K) * 3u);

  // 5) Una pasada: para cada tri_local, intenta unir con el tri que ya tenía la misma arista
  for (uint32_t tl = 0; tl < K; ++tl) {
    const uint32_t tg = loc2glob[tl];
    const uint32_t v0 = msg.navcels_v0[tg];
    const uint32_t v1 = msg.navcels_v1[tg];
    const uint32_t v2 = msg.navcels_v2[tg];

    const uint64_t e01 = edge_key(v0, v1);
    const uint64_t e12 = edge_key(v1, v2);
    const uint64_t e20 = edge_key(v2, v0);

    for (const uint64_t e : {e01, e12, e20}) {
      auto it = edge2tri.find(e);
      if (it == edge2tri.end()) {
        edge2tri.emplace(e, tl);
      } else {
        unite(tl, it->second);
      }
    }
  }

  // 6) Recolecta componentes: parent_local -> lista de tri_global
  std::unordered_map<uint32_t, std::vector<uint32_t>> comp;  // root -> tri_global list
  comp.reserve(K);
  for (uint32_t tl = 0; tl < K; ++tl) {
    uint32_t r = findp(tl);
    comp[r].push_back(loc2glob[tl]);
  }

  // 7) Pasa a vector y ordénalas por tamaño (desc)
  std::vector<std::vector<uint32_t>> comps;
  comps.reserve(comp.size());
  for (auto & kv : comp) comps.push_back(std::move(kv.second));
  std::sort(comps.begin(), comps.end(),
            [](const auto& A, const auto& B){ return A.size() > B.size(); });

  // 8) Emite las nuevas surfaces (sin recorte aquí; ya hiciste keep_top_surfaces_by_size)
  const std::string fid = msg.header.frame_id;
  std::vector<navmap_ros_interfaces::msg::NavMapSurface> out;
  out.reserve(comps.size());
  for (auto & C : comps) {
    navmap_ros_interfaces::msg::NavMapSurface s;
    s.frame_id = fid;
    s.navcels.assign(C.begin(), C.end()); // índices globales de triángulo
    out.push_back(std::move(s));
  }
  msg.surfaces.swap(out);
}

// ----------------- Variante Local Grow (con trazas + aceleraciones) -----------------

navmap::NavMap from_points_local_grow(
  const pcl::PointCloud<pcl::PointXYZ> & input_points,
  navmap_ros_interfaces::msg::NavMap & out_msg,
  BuildParams P)
{
  using std::cerr;

  out_msg = navmap_ros_interfaces::msg::NavMap();

  auto fmtf = [](float v){ std::ostringstream ss; ss<<std::fixed<<std::setprecision(3)<<v; return ss.str(); };

  cerr << "\n[local_grow] ====== BEGIN from_points_local_grow ======\n";
  cerr << "[local_grow] input_points: " << input_points.size() << "\n";
  cerr << "[local_grow] params:"
       << " resolution="      << P.resolution
       << " neighbor_radius=" << P.neighbor_radius
       << " k_neighbors="     << P.k_neighbors
       << " max_edge_len="    << P.max_edge_len
       << " max_dz="          << P.max_dz
       << " max_slope_deg="   << P.max_slope_deg
       << " min_area="        << P.min_area
       << " min_angle_deg="   << P.min_angle_deg
       << " max_surfaces="    << P.max_surfaces
       << "\n";

  if (input_points.size() < 3) {
    cerr << "[local_grow] Not enough input points (<3). RETURN.\n";
    return navmap::NavMap();
  }

  // Downsample que preserva plantas: top-Z por capa de Z (cluster en cada celda XY usando dz_merge)
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (P.resolution > 0.0f) {
    const float dz_merge = (P.max_dz > 0.0f ? P.max_dz : 0.30f);
    cloud = downsample_voxelize_topZ_layered(input_points, P.resolution, dz_merge);
  } else {
    cloud = input_points;
  }
  cerr << "[local_grow] cloud after downsample: " << cloud.size() << " points\n";
  if (cloud.size() < 3) {
    cerr << "[local_grow] Less than 3 points after downsample. RETURN.\n";
    return navmap::NavMap();
  }

  // KdTree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud.makeShared());
  cerr << "[local_grow] KdTree built.\n";

  const int N = static_cast<int>(cloud.size());
  std::vector<char> used_vertex(N, 0);
  std::vector<Eigen::Vector3i> triangles;
  triangles.reserve(N * 2);

  // Seeds en orden de Z ascendente
  std::vector<int> order(N);
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](int a, int b){ return cloud[a].z < cloud[b].z; });
  cerr << "[local_grow] seeds (ordered by Z): " << order.size() << "\n";

  // --- Estado global para toda la construcción ---
  std::unordered_set<TriKey, TriHasher>  tri_set_global;
  std::unordered_set<EdgeKey, EdgeHasher> edge_set_global; // requerido por try_add_triangle
  std::unordered_map<EdgeKey, uint16_t, EdgeHasher> edge_use_count;

  auto inc_edge = [&](const EdgeKey& e) -> uint16_t {
    auto it = edge_use_count.find(e);
    if (it == edge_use_count.end()) { edge_use_count.emplace(e, 1); return 1; }
    return ++(it->second);
  };
  auto edge_count = [&](const EdgeKey& e) -> uint16_t {
    auto it = edge_use_count.find(e);
    return (it == edge_use_count.end()) ? 0 : it->second;
  };

  std::queue<EdgeKey> frontier;
  std::unordered_set<EdgeKey, EdgeHasher> in_frontier;
  auto push_frontier_if_border = [&](const EdgeKey& e) {
    if (edge_count(e) == 1) {
      if (in_frontier.insert(e).second) {
        frontier.emplace(e);
      }
    }
  };

  // Contadores (trazas)
  struct RejectCounts { size_t edge_len=0,dz=0,xy=0,dup=0,geom=0,zwin_seed=0,zwin_bfs=0; };
  RejectCounts global_rej{};
  size_t global_tri_fan_attempts = 0, global_tri_fan_accept = 0;
  size_t global_tri_bfs_attempts = 0, global_tri_bfs_accept = 0;

  auto dist3f = [](const pcl::PointXYZ &A, const pcl::PointXYZ &B){
    const float dx=A.x-B.x, dy=A.y-B.y, dz=A.z-B.z; return std::sqrt(dx*dx+dy*dy+dz*dz);
  };
  auto distXY = [](const pcl::PointXYZ &A, const pcl::PointXYZ &B){
    const float dx=A.x-B.x, dy=A.y-B.y; return std::sqrt(dx*dx+dy*dy);
  };

  enum class Phase { FAN, BFS };
  auto precheck = [&](int i, int j, int k, Phase phase, RejectCounts & rej, bool &dup_out)->bool {
    TriKey tk = make_tri(i,j,k);
    if (tri_set_global.find(tk) != tri_set_global.end()) { rej.dup++; dup_out = true; return false; }
    dup_out = false;

    const auto &A = cloud[i], &B = cloud[j], &C = cloud[k];

    if (P.max_edge_len > 0.0f) {
      const float lAB = dist3f(A,B), lBC = dist3f(B,C), lCA = dist3f(C,A);
      if (lAB > P.max_edge_len || lBC > P.max_edge_len || lCA > P.max_edge_len) { rej.edge_len++; return false; }
    }

    if (P.max_dz > 0.0f) {
      const float dzAB = std::fabs(A.z - B.z);
      const float dzBC = std::fabs(B.z - C.z);
      const float dzCA = std::fabs(C.z - A.z);
      if (std::max({dzAB, dzBC, dzCA}) > P.max_dz) { rej.dz++; return false; }
    }

    if (P.neighbor_radius > 0.0f) {
      const float r = P.neighbor_radius;
      const float dABxy = distXY(A,B), dBCxy = distXY(B,C), dCAxy = distXY(C,A);
      bool bad_xy = (phase == Phase::FAN) ? (dABxy > r || dCAxy > r) : (dABxy > r || dBCxy > r || dCAxy > r);
      if (bad_xy) { rej.xy++; return false; }
    }

    return true;
  };

  // Ventanas Z: algo más holgadas hacia arriba para no cortar rampas
  const float z_window_seed = std::max(P.max_dz, 0.35f);
  const float z_window_bfs  = std::max(P.max_dz, 0.35f);
  const float z_window_down = 0.10f; // estricta hacia abajo en BFS

  std::vector<std::pair<size_t, size_t>> surf_tri_ranges; // [offset, count]

  size_t seed_idx_counter = 0;
  for (int seed_idx : order) {
    ++seed_idx_counter;
    if (used_vertex[seed_idx]) continue;

    const auto &S = cloud[seed_idx];
    cerr << "\n[local_grow] -- Seed #" << seed_idx_counter
         << " (idx=" << seed_idx << ") pos=("
         << fmtf(S.x) << "," << fmtf(S.y) << "," << fmtf(S.z) << ")\n";

    // Vecinos de la semilla
    std::vector<int> neigh_seed;
    {
      std::vector<int> inds; std::vector<float> dists;
      if (P.neighbor_radius > 0.0f) {
        const int found = kdtree.radiusSearch(S, P.neighbor_radius, inds, dists);
        cerr << "[local_grow]   radiusSearch: found=" << found << " (including self)\n";
        if (found > 0) for (int id : inds) if (id != seed_idx) neigh_seed.push_back(id);
      } else {
        const int K = std::max(8, P.k_neighbors);
        const int found = kdtree.nearestKSearch(S, K, inds, dists);
        cerr << "[local_grow]   kNN: K=" << K << " found=" << found << " (including self)\n";
        if (found > 0) for (int id : inds) if (id != seed_idx) neigh_seed.push_back(id);
      }
    }
    cerr << "[local_grow]   neigh_seed (raw, excl. self): " << neigh_seed.size() << "\n";

    // Filtros rápidos
    if (P.max_edge_len > 0.0f) {
      size_t before = neigh_seed.size();
      neigh_seed.erase(std::remove_if(neigh_seed.begin(), neigh_seed.end(),
        [&](int j){ const auto &Q = cloud[j]; if (!pcl::isFinite(Q)) return true; return dist3f(S,Q) > P.max_edge_len; }),
        neigh_seed.end());
      cerr << "[local_grow]   neigh_seed after edge_len filter: " << neigh_seed.size()
           << " (removed " << (before - neigh_seed.size()) << ")\n";
    }
    {
      size_t before = neigh_seed.size();
      neigh_seed.erase(std::remove_if(neigh_seed.begin(), neigh_seed.end(),
        [&](int j){ return std::fabs(cloud[j].z - S.z) > z_window_seed; }),
        neigh_seed.end());
      cerr << "[local_grow]   neigh_seed after Z-window (±"<< fmtf(z_window_seed) << "): "
           << neigh_seed.size() << " (removed " << (before - neigh_seed.size()) << ")\n";
      global_rej.zwin_seed += (before - neigh_seed.size());
    }

    if (neigh_seed.size() < 2) {
      used_vertex[seed_idx] = 1;
      cerr << "[local_grow]   Insufficient neighbors (<2) after filtering. Seed skipped.\n";
      continue;
    }

    // Orden angular en XY
    {
      const auto & Cc = cloud[seed_idx];
      std::sort(neigh_seed.begin(), neigh_seed.end(), [&](int a, int b){
        const float ax = cloud[a].x - Cc.x, ay = cloud[a].y - Cc.y;
        const float bx = cloud[b].x - Cc.x, by = cloud[b].y - Cc.y;
        return std::atan2(ay, ax) < std::atan2(by, bx);
      });
    }

    const size_t tri_off = triangles.size();

    RejectCounts comp_rej{};
    size_t comp_fan_attempts = 0, comp_fan_accept = 0;
    size_t comp_bfs_attempts = 0, comp_bfs_accept = 0;

    // --- FAN inicial ---
    cerr << "[local_grow]   Fan: pairs=" << (neigh_seed.size() > 0 ? (neigh_seed.size()-1) : 0) << "\n";
    for (size_t t = 0; t + 1 < neigh_seed.size(); ++t) {
      const int j = neigh_seed[t];
      const int k = neigh_seed[t+1];
      ++comp_fan_attempts; ++global_tri_fan_attempts;

      bool dup=false;
      if (!precheck(seed_idx, j, k, Phase::FAN, comp_rej, dup)) continue;

      if (try_add_triangle(seed_idx, j, k, cloud, P, tri_set_global, edge_set_global, triangles)) {
        ++comp_fan_accept; ++global_tri_fan_accept;

        // Actualiza conteos y frontera
        const EdgeKey e0 = make_edge(seed_idx, j);
        const EdgeKey e1 = make_edge(j, k);
        const EdgeKey e2 = make_edge(k, seed_idx);
        inc_edge(e0); inc_edge(e1); inc_edge(e2);
        push_frontier_if_border(e0);
        push_frontier_if_border(e1);
        push_frontier_if_border(e2);
      } else if (!dup) {
        comp_rej.geom++;
      }
    }
    if (neigh_seed.size() >= 3) {
      const int j = neigh_seed.back();
      const int k = neigh_seed.front();
      ++comp_fan_attempts; ++global_tri_fan_attempts;
      bool dup=false;
      if (precheck(seed_idx, j, k, Phase::FAN, comp_rej, dup)) {
        if (try_add_triangle(seed_idx, j, k, cloud, P, tri_set_global, edge_set_global, triangles)) {
          ++comp_fan_accept; ++global_tri_fan_accept;
          const EdgeKey e0 = make_edge(seed_idx, j);
          const EdgeKey e1 = make_edge(j, k);
          const EdgeKey e2 = make_edge(k, seed_idx);
          inc_edge(e0); inc_edge(e1); inc_edge(e2);
          push_frontier_if_border(e0);
          push_frontier_if_border(e1);
          push_frontier_if_border(e2);
        } else if (!dup) {
          comp_rej.geom++;
        }
      }
    }

    cerr << "[local_grow]   Fan result: attempts=" << comp_fan_attempts
         << " accepted=" << comp_fan_accept
         << " tri_now=" << (triangles.size() - tri_off) << "\n";
    if (triangles.size() == tri_off) {
      used_vertex[seed_idx] = 1;
      cerr << "[local_grow]   No triangle from fan. Seed skipped.\n";
      global_rej.edge_len += comp_rej.edge_len;
      global_rej.dz       += comp_rej.dz;
      global_rej.xy       += comp_rej.xy;
      global_rej.dup      += comp_rej.dup;
      global_rej.geom     += comp_rej.geom;
      global_rej.zwin_seed += comp_rej.zwin_seed;
      global_rej.zwin_bfs  += comp_rej.zwin_bfs;
      continue;
    }

    // Marca vértices usados de los nuevos triángulos
    for (size_t u = tri_off; u < triangles.size(); ++u) {
      used_vertex[triangles[u][0]] = 1;
      used_vertex[triangles[u][1]] = 1;
      used_vertex[triangles[u][2]] = 1;
    }

    cerr << "[local_grow]   Frontier initialized: " << in_frontier.size() << " edges\n";

    // --- BFS sobre aristas de frontera ---
    size_t bfs_iters = 0;
    while (!frontier.empty()) {
      EdgeKey e = frontier.front(); frontier.pop();
      in_frontier.erase(e);
      ++bfs_iters;

      // Sólo expandimos bordes actuales (conteo==1)
      if (edge_count(e) != 1) continue;

      const float z_mid = 0.5f * (cloud[e.a].z + cloud[e.b].z);

      // Vecinos de e.a y comprobación rápida con e.b (radio y ventana Z)
      std::vector<int> neigh_a;
      {
        std::vector<int> inds; std::vector<float> dists;
        if (P.neighbor_radius > 0.0f) {
          if (kdtree.radiusSearch(cloud[e.a], P.neighbor_radius, inds, dists) > 0) {
            for (int id : inds) if (id != e.a) neigh_a.push_back(id);
          }
        } else {
          const int K = std::max(8, P.k_neighbors);
          if (kdtree.nearestKSearch(cloud[e.a], K, inds, dists) > 0) {
            for (int id : inds) if (id != e.a) neigh_a.push_back(id);
          }
        }
      }

      for (int c : neigh_a) {
        if (c == e.a || c == e.b) continue;

        if (P.neighbor_radius > 0.0f) {
          const float dx = cloud[c].x - cloud[e.b].x;
          const float dy = cloud[c].y - cloud[e.b].y;
          if ((dx*dx + dy*dy) > P.neighbor_radius * P.neighbor_radius) continue;
        }

        const float z_half = std::max(P.max_dz, 0.35f); // o calcula en función de res & slope
        const float dz = cloud[c].z - z_mid;
        if (std::fabs(dz) > z_half) { ++global_rej.zwin_bfs; continue; }

        ++comp_bfs_attempts; ++global_tri_bfs_attempts;

        bool dup=false;
        if (!precheck(e.a, e.b, c, Phase::BFS, comp_rej, dup)) continue;

        if (try_add_triangle(e.a, e.b, c, cloud, P, tri_set_global, edge_set_global, triangles)) {
          ++comp_bfs_accept; ++global_tri_bfs_accept;

          // Actualiza conteos: el borde (a,b) deja de ser frontera (pasa a 2)
          const EdgeKey eab = make_edge(e.a, e.b);
          const EdgeKey eac = make_edge(e.a, c);
          const EdgeKey ecb = make_edge(c, e.b);

          inc_edge(eab);
          inc_edge(eac);
          inc_edge(ecb);

          // Encola SOLO los nuevos bordes de contorno
          push_frontier_if_border(eac);
          push_frontier_if_border(ecb);

          used_vertex[e.a] = used_vertex[e.b] = used_vertex[c] = 1;
        } else if (!dup) {
          comp_rej.geom++;
        }
      }

      if (bfs_iters % 512 == 0) {
        cerr << "[local_grow]   BFS iters=" << bfs_iters
             << " frontier_size~" << in_frontier.size()
             << " tri_total=" << triangles.size() << "\n";
      }
    }

    // Superficie final para esta semilla
    const size_t tri_cnt = triangles.size() - tri_off;
    if (tri_cnt > 0) {
      surf_tri_ranges.emplace_back(tri_off, tri_cnt);
      cerr << "[local_grow]   Component done: triangles=" << tri_cnt
           << " (fan acc=" << comp_fan_accept << "/" << comp_fan_attempts
           << ", bfs acc=" << comp_bfs_accept << "/" << comp_bfs_attempts << ")\n";

      // Corte temprano por número máximo de superficies
      if (P.max_surfaces > 0 &&
          surf_tri_ranges.size() >= static_cast<size_t>(P.max_surfaces)) {
        cerr << "[local_grow]   Reached max_surfaces (" << P.max_surfaces
             << "). Early stop of seed loop.\n";
        break;
      }
    }

    // Acumular contadores globales
    global_rej.edge_len += comp_rej.edge_len;
    global_rej.dz       += comp_rej.dz;
    global_rej.xy       += comp_rej.xy;
    global_rej.dup      += comp_rej.dup;
    global_rej.geom     += comp_rej.geom;
    global_rej.zwin_seed += comp_rej.zwin_seed;
    global_rej.zwin_bfs  += comp_rej.zwin_bfs;
  }

  // Resumen global
  cerr << "\n[local_grow] ====== SUMMARY ======\n";
  cerr << "[local_grow] total triangles: " << triangles.size() << "\n";
  cerr << "[local_grow] components (before prune): " << surf_tri_ranges.size() << "\n";
  cerr << "[local_grow] Fan: attempts=" << global_tri_fan_attempts
       << " accepted=" << global_tri_fan_accept << "\n";
  cerr << "[local_grow] BFS: attempts=" << global_tri_bfs_attempts
       << " accepted=" << global_tri_bfs_accept << "\n";
  cerr << "[local_grow] Rejects: edge=" << global_rej.edge_len
       << " dz=" << global_rej.dz
       << " xy=" << global_rej.xy
       << " dup=" << global_rej.dup
       << " geom=" << global_rej.geom
       << " zwin_seed=" << global_rej.zwin_seed
       << " zwin_bfs="  << global_rej.zwin_bfs
       << "\n";

  if (triangles.empty()) {
    cerr << "[local_grow] No triangles produced. RETURN empty.\n";
    return navmap::NavMap();
  }

  // Construcción NavMap + surfaces
  const std::string frame_id = "map";
  navmap_ros_interfaces::msg::NavMap msg_tmp;
  navmap::NavMap core;
  if (!build_navmap_from_mesh(cloud, triangles, frame_id, msg_tmp, &core)) {
    cerr << "[local_grow] build_navmap_from_mesh FAILED. RETURN empty.\n";
    return navmap::NavMap();
  }

  // Construye surfaces a partir de los rangos por componente
  msg_tmp.surfaces.clear();
  for (const auto & oc : surf_tri_ranges) {
    if (oc.second == 0) continue;
    navmap_ros_interfaces::msg::NavMapSurface s;
    s.frame_id = frame_id;
    s.navcels.resize(oc.second);
    for (size_t k = 0; k < oc.second; ++k) {
      s.navcels[k] = static_cast<uint32_t>(oc.first + k);
    }
    msg_tmp.surfaces.push_back(std::move(s));
  }

  // Mantener solo top-N superficies (si procede)
  rebuild_surfaces_by_connectivity(msg_tmp);
  keep_top_surfaces_by_size(msg_tmp, P.max_surfaces);

  out_msg = std::move(msg_tmp);
  cerr << "[local_grow] Surfaces emitted: " << out_msg.surfaces.size() << "\n";
  cerr << "[local_grow] ====== END from_points_local_grow ======\n\n";
  return from_msg(out_msg);
}



// ----------------- ROS PointCloud2 entry -----------------

navmap::NavMap from_pointcloud2(
  const sensor_msgs::msg::PointCloud2 & pc2,
  navmap_ros_interfaces::msg::NavMap & out_msg,
  BuildParams params)
{
  pcl::PointCloud<pcl::PointXYZ> input_points;
  pcl::fromROSMsg(pc2, input_points);

  // MODO por defecto: Banded Delaunay
  // return from_points(input_points, out_msg, params);

  // Para la variante local (con aceleraciones y trazas):
  return from_points_local_grow(input_points, out_msg, params);
}

} // namespace navmap_ros
