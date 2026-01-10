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

#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "navmap_ros_interfaces/msg/nav_map.hpp"
#include "navmap_ros_interfaces/msg/nav_map_layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/point_tests.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/kdtree/kdtree_flann.h"

namespace navmap_ros
{

using navmap_ros_interfaces::msg::NavMap;
using navmap_ros_interfaces::msg::NavMapLayer;
using navmap_ros_interfaces::msg::NavMapSurface;

// ----------------- Helpers (encoding) -----------------

static inline uint8_t occ_to_u8(int8_t v)
{
  if (v < 0) {return NO_INFORMATION;}
  if (v >= 100) {return LETHAL_OBSTACLE;}
  return static_cast<uint8_t>(std::lround((v / 100.0) * static_cast<double>(LETHAL_OBSTACLE)));
}

static inline int8_t u8_to_occ(uint8_t u)
{
  if (u == NO_INFORMATION) {return -1;}
  return static_cast<int8_t>(std::lround((u / static_cast<double>(LETHAL_OBSTACLE)) * 100.0));
}

static inline navmap::NavCelId tri_index_for_cell(uint32_t i, uint32_t j, uint32_t W)
{
  return static_cast<navmap::NavCelId>((j * W + i) * 2);
}

// ----------------- NavMap <-> ROS message -----------------

NavMap to_msg(const navmap::NavMap & nm, const std_msgs::msg::Header & header)
{
  NavMap out;
  out.header = header;

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

NavMap to_msg(const navmap::NavMap & nm)
{
  return to_msg(nm, std_msgs::msg::Header());
}

navmap::NavMap from_msg(const NavMap & msg, std_msgs::msg::Header & header)
{
  header = msg.header;
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

  // Fallback: create a single surface if none provided and triangles exist.
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

  nm.rebuild_geometry_accels();
  return nm;
}

navmap::NavMap from_msg(const NavMap & msg)
{
  std_msgs::msg::Header unused;
  return from_msg(msg, unused);
}

navmap_ros_interfaces::msg::NavMapLayer to_msg(
  const navmap::NavMap & nm,
  const std::string & layer_name,
  const std_msgs::msg::Header & header)
{
  navmap_ros_interfaces::msg::NavMapLayer msg;
  msg.header = header;
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

navmap_ros_interfaces::msg::NavMapLayer to_msg(
  const navmap::NavMap & nm,
  const std::string & layer_name)
{
  return to_msg(nm, layer_name, std_msgs::msg::Header());
}

void from_msg(
  const navmap_ros_interfaces::msg::NavMapLayer & msg,
  navmap::NavMap & nm,
  std_msgs::msg::Header & header)
{
  header = msg.header;
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
}

void from_msg(
  const navmap_ros_interfaces::msg::NavMapLayer & msg,
  navmap::NavMap & nm)
{
  std_msgs::msg::Header unused;
  from_msg(msg, nm, unused);
}

// ----------------- OccupancyGrid <-> NavMap -----------------

navmap::NavMap from_occupancy_grid(
  const nav_msgs::msg::OccupancyGrid & grid,
  std_msgs::msg::Header & header)
{
  header = grid.header;
  navmap::NavMap nm;

  const uint32_t W = grid.info.width;
  const uint32_t H = grid.info.height;
  const float res = grid.info.resolution;
  const auto & O = grid.info.origin;

  // Shared grid vertices
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

  // Triangles: 2 per cell
  nm.navcels.resize(static_cast<size_t>(2ull * W * H));
  size_t tidx = 0;
  for (uint32_t j = 0; j < H; ++j) {
    for (uint32_t i = 0; i < W; ++i) {
      {
        auto & c = nm.navcels[tidx++];
        c.v[0] = v_id(i, j);
        c.v[1] = v_id(i + 1, j);
        c.v[2] = v_id(i + 1, j + 1);
      }
      {
        auto & c = nm.navcels[tidx++];
        c.v[0] = v_id(i, j);
        c.v[1] = v_id(i + 1, j + 1);
        c.v[2] = v_id(i, j + 1);
      }
    }
  }

  // One surface listing all triangles
  nm.surfaces.resize(1);
  nm.surfaces[0].frame_id = grid.header.frame_id;
  nm.surfaces[0].navcels.resize(nm.navcels.size());
  for (navmap::NavCelId cid = 0; cid < nm.navcels.size(); ++cid) {
    nm.surfaces[0].navcels[cid] = cid;
  }

  nm.rebuild_geometry_accels();

  // Per-NavCel occupancy layer (U8)
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

navmap::NavMap from_occupancy_grid(const nav_msgs::msg::OccupancyGrid & grid)
{
  std_msgs::msg::Header unused;
  return from_occupancy_grid(grid, unused);
}

nav_msgs::msg::OccupancyGrid to_occupancy_grid(
  const navmap::NavMap & nm,
  const std_msgs::msg::Header & header)
{
  nav_msgs::msg::OccupancyGrid g;
  g.header = header;
  if (g.header.frame_id.empty() && !nm.surfaces.empty()) {
    g.header.frame_id = nm.surfaces[0].frame_id;
  }

  auto base = nm.layers.get("occupancy");
  if (!base || base->type() != navmap::LayerType::U8) {
    g.info.width = 0;
    g.info.height = 0;
    g.info.resolution = 1.0f;
    g.data.clear();
    return g;
  }
  auto occ = std::dynamic_pointer_cast<navmap::LayerView<uint8_t>>(base);

  // Fast regular-grid path
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
            const uint8_t u8 = (*occ)[tidx];
            g.data[idx_cell(i, j)] = u8_to_occ(u8);
            tidx += 2;
          }
        }
        return g;
      }
    }
  }

  // Generic sampling fallback
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

nav_msgs::msg::OccupancyGrid to_occupancy_grid(const navmap::NavMap & nm)
{
  std_msgs::msg::Header h;
  h.frame_id = (nm.surfaces.empty() ? std::string() : nm.surfaces[0].frame_id);
  return to_occupancy_grid(nm, h);
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

  // vertices
  out_msg.positions_x.reserve(cloud.size());
  out_msg.positions_y.reserve(cloud.size());
  out_msg.positions_z.reserve(cloud.size());
  for (const auto & p : cloud) {
    out_msg.positions_x.push_back(p.x);
    out_msg.positions_y.push_back(p.y);
    out_msg.positions_z.push_back(p.z);
  }

  // triangles (validate indices)
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
      return false;
    }
    out_msg.navcels_v0.push_back(static_cast<uint32_t>(i0));
    out_msg.navcels_v1.push_back(static_cast<uint32_t>(i1));
    out_msg.navcels_v2.push_back(static_cast<uint32_t>(i2));
  }

  // single surface with all triangles (required for BVH)
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

  // elevation layer (float32): mean Z per triangle
  {
    std::vector<float> elev;
    elev.reserve(triangles.size());
    for (const auto & t : triangles) {
      const int i0 = t[0], i1 = t[1], i2 = t[2];
      const float z0 = cloud[i0].z;
      const float z1 = cloud[i1].z;
      const float z2 = cloud[i2].z;
      elev.push_back((z0 + z1 + z2) / 3.0f);
    }

    navmap_ros_interfaces::msg::NavMapLayer layer;
    layer.name = "elevation";
    layer.type = 1; // F32
    layer.data_f32 = std::move(elev);
    out_msg.layers.push_back(std::move(layer));
  }

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
  // side lengths opposite to vertices A/B/C
  float a = (B - C).norm();
  float b = (C - A).norm();
  float c = (A - B).norm();

  const float eps = 1e-12f;
  a = std::max(a, eps);
  b = std::max(b, eps);
  c = std::max(c, eps);

  auto angle_from = [](float opp, float x, float y) -> float {
      float cosv = (x * x + y * y - opp * opp) / (2.0f * x * y);
      cosv = std::min(1.0f, std::max(-1.0f, cosv));
      return std::acos(cosv) * 180.0f / static_cast<float>(M_PI);
    };

  angA = angle_from(a, b, c);
  angB = angle_from(b, c, a);
  angC = angle_from(c, a, b);
}

// ----------------- Downsampling (only what is used) -----------------

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
    std::size_t h = 1469598103934665603ull;
    auto mix = [&](int k) {
        std::size_t x = static_cast<std::size_t>(k);
        h ^= x + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
      };
    mix(v.x); mix(v.y); mix(v.z);
    return h;
  }
};

// Layered “top-Z” downsampling: one point per XY voxel, using max Z per Z-cluster.
pcl::PointCloud<pcl::PointXYZ>
downsample_voxelize_topZ_layered(
  const pcl::PointCloud<pcl::PointXYZ> & input_points,
  float resolution,
  float dz_merge)
{
  pcl::PointCloud<pcl::PointXYZ> output;
  if (input_points.empty() || !(resolution > 0.0f)) {
    output.width = 0; output.height = 1; output.is_dense = true;
    return output;
  }
  if (!(dz_merge > 0.0f)) {dz_merge = 0.30f;}

  struct VoxelXY { int x, y; };
  struct HashXY
  {
    size_t operator()(const VoxelXY & v) const noexcept
    {
      return (static_cast<size_t>(v.x) * 73856093u) ^ (static_cast<size_t>(v.y) * 19349663u);
    }
  };
  struct EqXY
  {
    bool operator()(const VoxelXY & a, const VoxelXY & b) const noexcept
    {
      return a.x == b.x && a.y == b.y;
    }
  };

  std::unordered_map<VoxelXY, std::vector<int>, HashXY, EqXY> buckets;
  buckets.reserve(input_points.size() / 4);

  for (int i = 0; i < static_cast<int>(input_points.size()); ++i) {
    const auto & pt = input_points[i];
    if (!pcl::isFinite(pt)) {continue;}
    const int vx = static_cast<int>(std::floor(pt.x / resolution));
    const int vy = static_cast<int>(std::floor(pt.y / resolution));
    buckets[{vx, vy}].push_back(i);
  }

  output.points.reserve(buckets.size());

  for (auto & kv : buckets) {
    auto & idxs = kv.second;
    if (idxs.empty()) {continue;}

    std::sort(idxs.begin(), idxs.end(),
      [&](int a, int b){return input_points[a].z < input_points[b].z;});

    double sum_x = 0.0, sum_y = 0.0;
    float  z_max = -std::numeric_limits<float>::infinity();
    int    count = 0;
    float  last_z = input_points[idxs.front()].z;

    auto flush_cluster = [&](){
        if (count <= 0) {return;}
        const float cx = static_cast<float>(sum_x / count);
        const float cy = static_cast<float>(sum_y / count);
        output.emplace_back(cx, cy, z_max);
        sum_x = 0.0; sum_y = 0.0; z_max = -std::numeric_limits<float>::infinity(); count = 0;
      };

    for (int ii = 0; ii < static_cast<int>(idxs.size()); ++ii) {
      const auto & p = input_points[idxs[ii]];
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

// ----------------- Triangle acceptance filters -----------------
//
// Includes: max edge length, max |Δz|, XY neighbor radius, min area,
// min internal angle, and slope limit via n·z.

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

  // Slope limit: n·z >= cos(max_slope)
  const float cos_max_slope =
    std::cos(P.max_slope_deg * static_cast<float>(M_PI) / 180.0f);
  Eigen::Vector3f n = (b - a).cross(c - a);
  const float nn = n.norm();
  if (nn < 1e-9f) {return false;}
  n /= nn;
  if (n.dot(Eigen::Vector3f::UnitZ()) < cos_max_slope) {return false;}

  // Min internal angle
  float angA, angB, angC;
  triangle_angles_deg(a, b, c, angA, angB, angC);
  if (angA < P.min_angle_deg || angB < P.min_angle_deg || angC < P.min_angle_deg) {
    return false;
  }

  // Canonical orientation (normal facing +Z)
  if (n.dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
    std::swap(j, k);
  }

  tri_set.insert(tk);
  edge_set.insert(make_edge(i, j));
  edge_set.insert(make_edge(j, k));
  edge_set.insert(make_edge(k, i));
  tris.emplace_back(Triangle(i, j, k));
  return true;
}

// ----------------- Keep only largest surfaces -----------------

static void keep_top_surfaces_by_size(navmap_ros_interfaces::msg::NavMap & msg, int max_surfaces)
{
  if (max_surfaces <= 0) {return;}
  if (msg.surfaces.size() <= static_cast<size_t>(max_surfaces)) {return;}

  std::vector<size_t> ids(msg.surfaces.size());
  std::iota(ids.begin(), ids.end(), 0);

  std::sort(ids.begin(), ids.end(), [&](size_t a, size_t b){
      return msg.surfaces[a].navcels.size() > msg.surfaces[b].navcels.size();
  });

  std::vector<navmap_ros_interfaces::msg::NavMapSurface> kept;
  kept.reserve(static_cast<size_t>(max_surfaces));
  for (int i = 0; i < max_surfaces; ++i) {
    kept.push_back(std::move(msg.surfaces[ids[static_cast<size_t>(i)]]));
  }
  msg.surfaces.swap(kept);
}

// ----------------- Rebuild surfaces by triangle connectivity -----------------

static void rebuild_surfaces_by_connectivity(navmap_ros_interfaces::msg::NavMap & msg)
{
  const size_t ntris = msg.navcels_v0.size();
  if (ntris == 0) {msg.surfaces.clear(); return;}

  // Triangles to consider: those referenced by current surfaces, or all if none.
  std::vector<char> keep(ntris, 0);
  size_t kept_count = 0;
  if (!msg.surfaces.empty()) {
    for (const auto & s : msg.surfaces) {
      for (uint32_t cid : s.navcels) {
        if (cid < ntris && !keep[cid]) {keep[cid] = 1; ++kept_count;}
      }
    }
  } else {
    kept_count = ntris;
    std::fill(keep.begin(), keep.end(), 1);
  }
  if (kept_count == 0) {msg.surfaces.clear(); return;}

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

  // Union-Find
  std::vector<uint32_t> parent(K), rankv(K, 0);
  std::iota(parent.begin(), parent.end(), 0);
  auto findp = [&](uint32_t x) {
      while (parent[x] != x) {parent[x] = parent[parent[x]]; x = parent[x];}
      return x;
    };
  auto unite = [&](uint32_t a, uint32_t b) {
      a = findp(a); b = findp(b);
      if (a == b) {return;}
      if (rankv[a] < rankv[b]) {std::swap(a, b);}
      parent[b] = a;
      if (rankv[a] == rankv[b]) {++rankv[a];}
    };

  struct U64Hash
  {
    size_t operator()(const uint64_t & k) const noexcept
    {
      return static_cast<size_t>(k ^ (k >> 33));
    }
  };
  auto edge_key = [](uint32_t a, uint32_t b) -> uint64_t {
      const uint64_t mn = (a < b) ? a : b;
      const uint64_t mx = (a < b) ? b : a;
      return (mn << 32) | mx;
    };

  std::unordered_map<uint64_t, uint32_t, U64Hash> edge2tri;
  edge2tri.reserve(static_cast<size_t>(K) * 3u);

  // Single pass: join triangles sharing an edge
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

  // Collect components
  std::unordered_map<uint32_t, std::vector<uint32_t>> comp;
  comp.reserve(K);
  for (uint32_t tl = 0; tl < K; ++tl) {
    uint32_t r = findp(tl);
    comp[r].push_back(loc2glob[tl]);
  }

  std::vector<std::vector<uint32_t>> comps;
  comps.reserve(comp.size());
  for (auto & kv : comp) {
    comps.push_back(std::move(kv.second));
  }
  std::sort(comps.begin(), comps.end(),
    [](const auto & A, const auto & B){return A.size() > B.size();});

  const std::string fid = msg.header.frame_id;
  std::vector<navmap_ros_interfaces::msg::NavMapSurface> out;
  out.reserve(comps.size());
  for (auto & C : comps) {
    navmap_ros_interfaces::msg::NavMapSurface s;
    s.frame_id = fid;
    s.navcels.assign(C.begin(), C.end());
    out.push_back(std::move(s));
  }
  msg.surfaces.swap(out);
}

// ----------------- Local Grow builder (renamed: from_points) -----------------

navmap::NavMap from_points(
  const pcl::PointCloud<pcl::PointXYZ> & input_points,
  navmap_ros_interfaces::msg::NavMap & out_msg,
  BuildParams P)
{
  out_msg = navmap_ros_interfaces::msg::NavMap();

  if (input_points.size() < 3) {
    return navmap::NavMap();
  }

  // Downsample that preserves floors: top-Z per Z-layer in each XY cell
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (P.resolution > 0.0f) {
    const float dz_merge = (P.max_dz > 0.0f ? P.max_dz : 0.30f);
    cloud = downsample_voxelize_topZ_layered(input_points, P.resolution, dz_merge);
  } else {
    cloud = input_points;
  }
  if (cloud.size() < 3) {
    return navmap::NavMap();
  }

  // Neighborhood structure
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud.makeShared());

  const int N = static_cast<int>(cloud.size());
  std::vector<char> used_vertex(N, 0);
  std::vector<Eigen::Vector3i> triangles;
  triangles.reserve(N * 2);

  // Seeds in ascending Z
  std::vector<int> order(N);
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
    [&](int a, int b){return cloud[a].z < cloud[b].z;});

  // Global state
  std::unordered_set<TriKey, TriHasher> tri_set_global;
  std::unordered_set<EdgeKey, EdgeHasher> edge_set_global;
  std::unordered_map<EdgeKey, uint16_t, EdgeHasher> edge_use_count;

  auto inc_edge = [&](const EdgeKey & e) -> uint16_t {
      auto it = edge_use_count.find(e);
      if (it == edge_use_count.end()) {edge_use_count.emplace(e, 1); return 1;}
      return ++(it->second);
    };
  auto edge_count = [&](const EdgeKey & e) -> uint16_t {
      auto it = edge_use_count.find(e);
      return (it == edge_use_count.end()) ? 0 : it->second;
    };

  std::queue<EdgeKey> frontier;
  std::unordered_set<EdgeKey, EdgeHasher> in_frontier;
  auto push_frontier_if_border = [&](const EdgeKey & e) {
      if (edge_count(e) == 1) {
        if (in_frontier.insert(e).second) {
          frontier.emplace(e);
        }
      }
    };

  auto dist3f = [](const pcl::PointXYZ & A, const pcl::PointXYZ & B){
      const float dx = A.x - B.x, dy = A.y - B.y, dz = A.z - B.z;
      return std::sqrt(dx * dx + dy * dy + dz * dz);
    };
  auto distXY = [](const pcl::PointXYZ & A, const pcl::PointXYZ & B){
      const float dx = A.x - B.x, dy = A.y - B.y; return std::sqrt(dx * dx + dy * dy);
    };

  enum class Phase { FAN, BFS };
  struct RejectCounts { size_t edge_len = 0, dz = 0, xy = 0, dup = 0, geom = 0, zwin_seed = 0,
      zwin_bfs = 0; };
  auto precheck = [&](int i, int j, int k, Phase phase, RejectCounts & rej, bool & dup_out)->bool {
      TriKey tk = make_tri(i, j, k);
      if (tri_set_global.find(tk) != tri_set_global.end()) {
        rej.dup++; dup_out = true; return false;
      }
      dup_out = false;

      const auto & A = cloud[i], & B = cloud[j], & C = cloud[k];

      if (P.max_edge_len > 0.0f) {
        const float lAB = dist3f(A, B), lBC = dist3f(B, C), lCA = dist3f(C, A);
        if (lAB > P.max_edge_len || lBC > P.max_edge_len || lCA > P.max_edge_len) {
          rej.edge_len++; return false;
        }
      }

      if (P.max_dz > 0.0f) {
        const float dzAB = std::fabs(A.z - B.z);
        const float dzBC = std::fabs(B.z - C.z);
        const float dzCA = std::fabs(C.z - A.z);
        if (std::max({dzAB, dzBC, dzCA}) > P.max_dz) {rej.dz++; return false;}
      }

      if (P.neighbor_radius > 0.0f) {
        const float r = P.neighbor_radius;
        const float dABxy = distXY(A, B), dBCxy = distXY(B, C), dCAxy = distXY(C, A);
        bool bad_xy = (phase == Phase::FAN) ? (dABxy > r || dCAxy > r) : (dABxy > r || dBCxy > r ||
          dCAxy > r);
        if (bad_xy) {rej.xy++; return false;}
      }

      return true;
    };

  // Z windows around the expanding boundary (keeps ramps but avoids jumps)
  const float z_window_seed = std::max(P.max_dz, 0.35f);

  std::vector<std::pair<size_t, size_t>> surf_tri_ranges; // [offset, count]

  RejectCounts global_rej{}; // still used to track precheck stats (no logging)

  for (int seed_idx : order) {
    if (used_vertex[seed_idx]) {continue;}

    // Neighborhood of the seed
    std::vector<int> neigh_seed;
    {
      std::vector<int> inds; std::vector<float> dists;
      if (P.neighbor_radius > 0.0f) {
        if (kdtree.radiusSearch(cloud[seed_idx], P.neighbor_radius, inds, dists) > 0) {
          for (int id : inds) {
            if (id != seed_idx) {
              neigh_seed.push_back(id);
            }
          }
        }
      } else {
        const int K = std::max(8, P.k_neighbors);
        if (kdtree.nearestKSearch(cloud[seed_idx], K, inds, dists) > 0) {
          for (int id : inds) {
            if (id != seed_idx) {
              neigh_seed.push_back(id);
            }
          }
        }
      }
    }

    // Quick filters
    if (P.max_edge_len > 0.0f) {
      neigh_seed.erase(std::remove_if(neigh_seed.begin(), neigh_seed.end(),
        [&](int j){
          const auto & Q = cloud[j]; if (!pcl::isFinite(Q)) {
            return true;
          }
          return dist3f(cloud[seed_idx], Q) > P.max_edge_len;
                                                                                                                                       }),
        neigh_seed.end());
    }
    {
      neigh_seed.erase(std::remove_if(neigh_seed.begin(), neigh_seed.end(),
        [&](int j){return std::fabs(cloud[j].z - cloud[seed_idx].z) > z_window_seed;}),
        neigh_seed.end());
    }

    if (neigh_seed.size() < 2) {
      used_vertex[seed_idx] = 1;
      continue;
    }

    // Angular sort in XY
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
    size_t comp_fan_accept = 0;
    size_t comp_bfs_accept = 0;

    // Initial fan
    for (size_t t = 0; t + 1 < neigh_seed.size(); ++t) {
      const int j = neigh_seed[t];
      const int k = neigh_seed[t + 1];
      bool dup = false;
      if (!precheck(seed_idx, j, k, Phase::FAN, comp_rej, dup)) {continue;}

      if (try_add_triangle(seed_idx, j, k, cloud, P, tri_set_global, edge_set_global, triangles)) {
        ++comp_fan_accept;

        // Update counts and frontier
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
      bool dup = false;
      if (precheck(seed_idx, j, k, Phase::FAN, comp_rej, dup)) {
        if (try_add_triangle(seed_idx, j, k, cloud, P, tri_set_global, edge_set_global,
            triangles))
        {
          ++comp_fan_accept;
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

    if (triangles.size() == tri_off) {
      used_vertex[seed_idx] = 1;
      global_rej.edge_len += comp_rej.edge_len;
      global_rej.dz += comp_rej.dz;
      global_rej.xy += comp_rej.xy;
      global_rej.dup += comp_rej.dup;
      global_rej.geom += comp_rej.geom;
      global_rej.zwin_seed += comp_rej.zwin_seed;
      global_rej.zwin_bfs += comp_rej.zwin_bfs;
      continue;
    }

    // Mark vertices used by the new triangles
    for (size_t u = tri_off; u < triangles.size(); ++u) {
      used_vertex[triangles[u][0]] = 1;
      used_vertex[triangles[u][1]] = 1;
      used_vertex[triangles[u][2]] = 1;
    }

    // BFS expansion over border edges
    while (!frontier.empty()) {
      EdgeKey e = frontier.front(); frontier.pop();
      in_frontier.erase(e);

      // Expand only current border edges (count == 1)
      if (edge_count(e) != 1) {continue;}

      const float z_mid = 0.5f * (cloud[e.a].z + cloud[e.b].z);

      // Neighbors of e.a (filtered by radius and Z window vs e.b)
      std::vector<int> neigh_a;
      {
        std::vector<int> inds; std::vector<float> dists;
        if (P.neighbor_radius > 0.0f) {
          if (kdtree.radiusSearch(cloud[e.a], P.neighbor_radius, inds, dists) > 0) {
            for (int id : inds) {
              if (id != e.a) {
                neigh_a.push_back(id);
              }
            }
          }
        } else {
          const int K = std::max(8, P.k_neighbors);
          if (kdtree.nearestKSearch(cloud[e.a], K, inds, dists) > 0) {
            for (int id : inds) {
              if (id != e.a) {
                neigh_a.push_back(id);
              }
            }
          }
        }
      }

      for (int c : neigh_a) {
        if (c == e.a || c == e.b) {continue;}

        if (P.neighbor_radius > 0.0f) {
          const float dx = cloud[c].x - cloud[e.b].x;
          const float dy = cloud[c].y - cloud[e.b].y;
          if ((dx * dx + dy * dy) > P.neighbor_radius * P.neighbor_radius) {continue;}
        }

        const float z_half = std::max(P.max_dz, 0.35f);
        const float dz = cloud[c].z - z_mid;
        if (std::fabs(dz) > z_half) {++global_rej.zwin_bfs; continue;}

        bool dup = false;
        if (!precheck(e.a, e.b, c, Phase::BFS, comp_rej, dup)) {continue;}

        if (try_add_triangle(e.a, e.b, c, cloud, P, tri_set_global, edge_set_global, triangles)) {
          ++comp_bfs_accept;

          const EdgeKey eab = make_edge(e.a, e.b);
          const EdgeKey eac = make_edge(e.a, c);
          const EdgeKey ecb = make_edge(c, e.b);

          inc_edge(eab);
          inc_edge(eac);
          inc_edge(ecb);

          push_frontier_if_border(eac);
          push_frontier_if_border(ecb);

          used_vertex[e.a] = used_vertex[e.b] = used_vertex[c] = 1;
        } else if (!dup) {
          comp_rej.geom++;
        }
      }
    }

    // Component done
    const size_t tri_cnt = triangles.size() - tri_off;
    if (tri_cnt > 0) {
      surf_tri_ranges.emplace_back(tri_off, tri_cnt);
    }

    // Accumulate global counters (used only for internal accounting)
    global_rej.edge_len += comp_rej.edge_len;
    global_rej.dz += comp_rej.dz;
    global_rej.xy += comp_rej.xy;
    global_rej.dup += comp_rej.dup;
    global_rej.geom += comp_rej.geom;
    global_rej.zwin_seed += comp_rej.zwin_seed;
    global_rej.zwin_bfs += comp_rej.zwin_bfs;
  }

  if (triangles.empty()) {
    return navmap::NavMap();
  }

  // Build NavMap + surfaces
  const std::string frame_id = "map";
  navmap_ros_interfaces::msg::NavMap msg_tmp;
  navmap::NavMap core;
  if (!build_navmap_from_mesh(cloud, triangles, frame_id, msg_tmp, &core)) {
    return navmap::NavMap();
  }

  // Build surfaces from component ranges
  msg_tmp.surfaces.clear();
  for (const auto & oc : surf_tri_ranges) {
    if (oc.second == 0) {continue;}
    navmap_ros_interfaces::msg::NavMapSurface s;
    s.frame_id = frame_id;
    s.navcels.resize(oc.second);
    for (size_t k = 0; k < oc.second; ++k) {
      s.navcels[k] = static_cast<uint32_t>(oc.first + k);
    }
    msg_tmp.surfaces.push_back(std::move(s));
  }

  rebuild_surfaces_by_connectivity(msg_tmp);
  keep_top_surfaces_by_size(msg_tmp, P.max_surfaces);

  out_msg = std::move(msg_tmp);
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

  return from_points(input_points, out_msg, params);
}

} // namespace navmap_ros
