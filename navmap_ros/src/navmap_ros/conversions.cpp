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
static inline float tri_slope_deg(
  const Eigen::Vector3f & a,
  const Eigen::Vector3f & b,
  const Eigen::Vector3f & c)
{
  Eigen::Vector3f n = (b - a).cross(c - a);
  float nn = n.norm();
  if (nn < 1e-9f) {return 0.0f;}
  float cos_theta = std::abs(n.normalized().dot(Eigen::Vector3f::UnitZ())); // cosine w.r.t. vertical
  cos_theta = std::clamp(cos_theta, 0.0f, 1.0f);
  float theta = std::acos(cos_theta); // angle w.r.t. vertical
  return theta * 180.0f / static_cast<float>(M_PI);
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

// Local PCA to obtain a tangent plane at v.
// Returns two orthogonal axes t1, t2 on the tangent plane.
// If PCA is unreliable (too few points), fall back to a stable orthonormal pair.
inline void local_tangent_basis(
  const std::vector<Eigen::Vector3f> & nbrs,
  Eigen::Vector3f & t1, Eigen::Vector3f & t2)
{
  if (nbrs.size() < 3) {
    t1 = Eigen::Vector3f::UnitX();
    t2 = Eigen::Vector3f::UnitY();
    return;
  }

  // Center
  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  for (auto & p : nbrs) {
    mean += p;
  }
  mean /= static_cast<float>(nbrs.size());

  // Covariance
  Eigen::Matrix3f C = Eigen::Matrix3f::Zero();
  for (auto & p : nbrs) {
    Eigen::Vector3f d = p - mean;
    C += d * d.transpose();
  }
  C /= static_cast<float>(nbrs.size());

  // Eigenvectors
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(C);
  // Ascending eigenvalues: column 0 ~ smallest (approximate normal)
  Eigen::Vector3f e1 = es.eigenvectors().col(1);
  Eigen::Vector3f e2 = es.eigenvectors().col(2);

  // Tangent plane basis {e1, e2}
  t1 = e1.normalized();
  t2 = (e2 - e2.dot(t1) * t1).normalized();
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

// Sort neighbors by angle on the tangent plane at v
inline void sort_neighbors_angular(
  const Eigen::Vector3f & vpos,
  const std::vector<std::pair<int, Eigen::Vector3f>> & nbrs,  // (idx, pos)
  std::vector<int> & out_ordered)
{
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(nbrs.size() + 1);
  pts.push_back(vpos);
  for (auto & it : nbrs) {
    pts.push_back(it.second);
  }

  Eigen::Vector3f t1, t2;
  local_tangent_basis(pts, t1, t2);

  std::vector<std::pair<float, int>> ang_idx;
  ang_idx.reserve(nbrs.size());
  for (auto & it : nbrs) {
    Eigen::Vector3f d = it.second - vpos;
    float x = d.dot(t1);
    float y = d.dot(t2);
    float ang = std::atan2(y, x); // -pi..pi
    ang_idx.emplace_back(ang, it.first);
  }

  std::sort(ang_idx.begin(), ang_idx.end(),
    [](auto & a, auto & b){return a.first < b.first;});

  out_ordered.clear();
  out_ordered.reserve(ang_idx.size());
  for (auto & ai : ang_idx) {
    out_ordered.push_back(ai.second);
  }
}

// Attempt to add triangle (i,j,k) under the given constraints
// (NOW includes max_dz and neighbor_radius constraints)
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

  // Max |Δz| per edge (avoid bridges between floors)
  if (P.max_dz > 0.0f) {
    const float dzAB = std::fabs(A.z - B.z);
    const float dzBC = std::fabs(B.z - C.z);
    const float dzCA = std::fabs(C.z - A.z);
    if (std::max({dzAB, dzBC, dzCA}) > P.max_dz) {return false;}
  }

  // Neighbor radius (XY) as lateral constraint
  if (P.neighbor_radius > 0.0f) {
    auto distXY = [](const pcl::PointXYZ & Pp, const pcl::PointXYZ & Qq) {
      const float dx = Pp.x - Qq.x, dy = Pp.y - Qq.y;
      return std::sqrt(dx*dx + dy*dy);
    };
    if (distXY(A,B) > P.neighbor_radius ||
        distXY(B,C) > P.neighbor_radius ||
        distXY(C,A) > P.neighbor_radius) {return false;}
  }

  Eigen::Vector3f a(A.x, A.y, A.z);
  Eigen::Vector3f b(B.x, B.y, B.z);
  Eigen::Vector3f c(C.x, C.y, C.z);

  // Min area
  if (tri_area(a, b, c) < std::max(P.min_area, 1e-9f)) {return false;}

  // Max slope
  float slope = tri_slope_deg(a, b, c);
  if (slope > P.max_slope_deg) {return false;}

  // Min angle at each vertex
  float angA, angB, angC;
  triangle_angles_deg(a, b, c, angA, angB, angC);
  if (angA < P.min_angle_deg || angB < P.min_angle_deg || angC < P.min_angle_deg) {
    return false;
  }

  // Orient consistently (up-facing)
  {
    const Eigen::Vector3f up = Eigen::Vector3f::UnitZ();
    Eigen::Vector3f n = (b - a).cross(c - a);
    if (n.norm() < 1e-9f) {return false;}
    if (n.dot(up) < 0.0f) {
      std::swap(j, k);
    }
  }

  // Accept
  tri_set.insert(tk);
  edge_set.insert(make_edge(i, j));
  edge_set.insert(make_edge(j, k));
  edge_set.insert(make_edge(k, i));
  tris.emplace_back(Triangle(i, j, k));
  return true;
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


// Compute voxel index with an offset (used for shifted grids)
static inline Voxel voxel_index_of_offset(
  const pcl::PointXYZ & p, float res, float ox, float oy, float oz)
{
  return Voxel{
    static_cast<int>(std::floor((p.x - ox) / res)),
    static_cast<int>(std::floor((p.y - oy) / res)),
    static_cast<int>(std::floor((p.z - oz) / res))
  };
}

// Single voxelization pass with offset; returns centroids per voxel
static std::vector<pcl::PointXYZ> voxel_pass_offset(
  const pcl::PointCloud<pcl::PointXYZ> & in, float res, float ox, float oy, float oz)
{
  std::unordered_map<Voxel, VoxelAccum, VoxelHash> map;
  map.reserve(in.size() / 2 + 1);

  for (const auto & pt : in.points) {
    if (!pcl::isFinite(pt)) {continue;}
    Voxel v = voxel_index_of_offset(pt, res, ox, oy, oz);
    auto & acc = map[v];
    acc.sum_x += pt.x;
    acc.sum_y += pt.y;
    acc.sum_z += pt.z;
    acc.count += 1;
  }

  std::vector<pcl::PointXYZ> out;
  out.reserve(map.size());
  for (const auto & kv : map) {
    const VoxelAccum & acc = kv.second;
    const float inv = acc.count > 0 ? 1.0f / static_cast<float>(acc.count) : 0.0f;
    out.emplace_back(acc.sum_x * inv, acc.sum_y * inv, acc.sum_z * inv);
  }
  return out;
}

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
  auto centroids_a = voxel_pass_offset(input_points, r, 0.0f, 0.0f, 0.0f);

  // Pass B: grid shifted by half resolution
  auto centroids_b = voxel_pass_offset(input_points, r, half, half, half);

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

// ----------------- Crecimiento local desde semilla -----------------

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

  std::vector<int> nbr_indices;
  std::vector<float> nbr_dists;

  while (!frontier.empty()) {
    int v = frontier.front(); frontier.pop();
    const auto & V = cloud[v];

    if (!pcl::isFinite(V)) {continue;}

    // 1) Neighborhood
    nbr_indices.clear(); nbr_dists.clear();
    int found = 0;
    if (P.use_radius) {
      found = kdtree.radiusSearch(V, P.neighbor_radius, nbr_indices, nbr_dists);
    } else {
      found = kdtree.nearestKSearch(V, P.k_neighbors, nbr_indices, nbr_dists);
    }
    if (found <= 1) {continue;} // only itself

    // 2) Filter by edge length and drop v itself
    std::vector<std::pair<int, Eigen::Vector3f>> candidates;
    candidates.reserve(found);
    for (int idx : nbr_indices) {
      if (idx == v) {continue;}
      const auto & Pn = cloud[idx];
      if (!pcl::isFinite(Pn)) {continue;}
      if (P.max_edge_len <= 0.0f || dist3(V, Pn) <= P.max_edge_len) {
        candidates.emplace_back(idx, Eigen::Vector3f(Pn.x, Pn.y, Pn.z));
      }
    }
    if (candidates.size() < 2) {continue;}

    // 3) Angular order on the local tangent plane at v
    Eigen::Vector3f vpos(V.x, V.y, V.z);
    std::vector<int> ordered;
    sort_neighbors_angular(vpos, candidates, ordered);

    // 4) Try fan triangles from consecutive pairs
    for (size_t t = 0; t + 1 < ordered.size(); ++t) {
      int i = v;
      int j = ordered[t];
      int k = ordered[t + 1];

      if (try_add_triangle(i, j, k, cloud, P, tri_set, edge_set, tris)) {
        if (!seen.count(j)) {frontier.push(j); seen.insert(j);}
        if (!seen.count(k)) {frontier.push(k); seen.insert(k);}
      }
    }
    // Optional fan closure
    if (ordered.size() >= 2) {
      int j0 = ordered.front(), k0 = ordered.back();
      try_add_triangle(v, k0, j0, cloud, P, tri_set, edge_set, tris);
    }
  }

  return tris;
}

// ----------------- Banded Delaunay (gap-based) -----------------

// Separa la nube en "bandas" siguiendo saltos en Z:
// - Ordena por Z y empieza nueva banda cuando (z_i - z_{i-1}) > gap_z.
// - Útil para plantas en edificios (saltos grandes) sin romper rampas suaves.
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
    cloud = downsample_voxelize_avgZ(input_points, params.resolution);
  } else {
    cloud = input_points;
  }
  if (cloud.size() < 3) {
    return navmap::NavMap();
  }

  // 2) Bandas por SALTOS en Z
  //    Para parkings típicos (2.5–3.0 m entre plantas), gap_z=1.0 separa bien.
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
  // Si tu builder reordena, puedes re-clasificar por Z media (ver variante en mensajes previos).
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
    // Guardamos el punto de mayor Z visto en el voxel
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
    // emitimos el “techo” del voxel
    output.emplace_back(a.cx, a.cy, a.zmax);
  }

  output.width = static_cast<uint32_t>(output.points.size());
  output.height = 1;
  output.is_dense = true;
  return output;
}


// ----------------- Variante Local Grow (opcional) -----------------

navmap::NavMap from_points_local_grow(
  const pcl::PointCloud<pcl::PointXYZ> & input_points,
  navmap_ros_interfaces::msg::NavMap & out_msg,
  BuildParams P)
{
  using std::cerr;
  using std::endl;

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
       << "\n";

  if (input_points.size() < 3) {
    cerr << "[local_grow] Not enough input points (<3). RETURN.\n";
    return navmap::NavMap();
  }

  // 0) Downsample que no colapsa plantas
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (P.resolution > 0.0f) {
    cloud = downsample_voxelize_topZ(input_points, P.resolution);
  } else {
    cloud = input_points;
  }
  cerr << "[local_grow] cloud after downsample: " << cloud.size() << " points\n";
  if (cloud.size() < 3) {
    cerr << "[local_grow] Less than 3 points after downsample. RETURN.\n";
    return navmap::NavMap();
  }

  // 1) KdTree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud.makeShared());
  cerr << "[local_grow] KdTree built.\n";

  const int N = static_cast<int>(cloud.size());
  std::vector<char> used_vertex(N, 0);
  std::vector<Eigen::Vector3i> triangles;
  triangles.reserve(N * 3);

  // Seeds en orden de Z ascendente
  std::vector<int> order(N);
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](int a, int b){ return cloud[a].z < cloud[b].z; });
  cerr << "[local_grow] seeds (ordered by Z): " << order.size() << "\n";

  // Rangos por superficie
  std::vector<std::pair<size_t, size_t>> surf_tri_ranges;

  // Estructuras temporales por componente
  std::unordered_set<TriKey, TriHasher> tri_set;
  std::unordered_set<EdgeKey, EdgeHasher> edge_set;

  std::queue<EdgeKey> frontier;

  // Contadores globales
  struct RejectCounts {
    size_t edge_len = 0;
    size_t dz = 0;
    size_t xy = 0;
    size_t dup = 0;
    size_t geom = 0; // min_area / slope / min_angle / degenerado
    size_t zwin_seed = 0; // NUEVO: vecinos filtrados por ventana Z en fan
    size_t zwin_bfs  = 0; // NUEVO: candidatos filtrados por ventana Z en BFS
  } global_rej{};

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
    // Duplicado
    TriKey tk = make_tri(i,j,k);
    if (tri_set.find(tk) != tri_set.end()) { rej.dup++; dup_out = true; return false; }
    dup_out = false;

    const auto &A = cloud[i], &B = cloud[j], &C = cloud[k];

    // Max edge (3D)
    if (P.max_edge_len > 0.0f) {
      const float lAB = dist3f(A,B), lBC = dist3f(B,C), lCA = dist3f(C,A);
      if (lAB > P.max_edge_len || lBC > P.max_edge_len || lCA > P.max_edge_len) {
        rej.edge_len++;
        // cerr << "[local_grow]     REJECT(edge_len) |AB|="<<fmtf(lAB)<<" |BC|="<<fmtf(lBC)<<" |CA|="<<fmtf(lCA)<<"\n";
        return false;
      }
    }

    // Max dz por arista
    if (P.max_dz > 0.0f) {
      const float dzAB = std::fabs(A.z - B.z);
      const float dzBC = std::fabs(B.z - C.z);
      const float dzCA = std::fabs(C.z - A.z);
      const float dzmax = std::max({dzAB, dzBC, dzCA});
      if (dzmax > P.max_dz) {
        rej.dz++;
        // cerr << "[local_grow]     REJECT(dz) dzAB="<<fmtf(dzAB)<<" dzBC="<<fmtf(dzBC)<<" dzCA="<<fmtf(dzCA)<<"\n";
        return false;
      }
    }

    // neighbor_radius (XY)
    if (P.neighbor_radius > 0.0f) {
      const float r = P.neighbor_radius;
      const float dABxy = distXY(A,B);
      const float dBCxy = distXY(B,C);
      const float dCAxy = distXY(C,A);

      bool bad_xy = false;
      if (phase == Phase::FAN) {
        // en FAN exigimos radio sólo para las aristas con la semilla (i)
        if (dABxy > r || dCAxy > r) bad_xy = true;
      } else {
        if (dABxy > r || dBCxy > r || dCAxy > r) bad_xy = true;
      }

      if (bad_xy) {
        rej.xy++;
        // cerr << "[local_grow]     REJECT(xy "<<(phase==Phase::FAN?"FAN":"BFS")<<") dAB="<<fmtf(dABxy)<<" dBC="<<fmtf(dBCxy)<<" dCA="<<fmtf(dCAxy)<<" (r="<<fmtf(r)<<")\n";
        return false;
      }
    }

    return true;
  };

  // Ventanas Z (ligeramente más holgadas que max_dz para inicio de rampa)
  const float z_window_seed = std::max(P.max_dz, 0.35f);
  const float z_window_bfs  = std::max(P.max_dz, 0.35f);

  // Bucle de semillas
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
      std::vector<int> inds;
      std::vector<float> dists;
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

    // Filtro rápido por edge_len (semilla–vecino) y finitos
    if (P.max_edge_len > 0.0f) {
      size_t before = neigh_seed.size();
      neigh_seed.erase(std::remove_if(neigh_seed.begin(), neigh_seed.end(),
        [&](int j){
          const auto &Q = cloud[j];
          if (!pcl::isFinite(Q)) return true;
          return dist3f(S, Q) > P.max_edge_len;
        }), neigh_seed.end());
      cerr << "[local_grow]   neigh_seed after edge_len filter: " << neigh_seed.size()
           << " (removed " << (before - neigh_seed.size()) << ")\n";
    }

    // *** NUEVO ***: Filtro por ventana Z respecto a la semilla
    {
      size_t before = neigh_seed.size();
      size_t dump_max = 3, dumped = 0;
      neigh_seed.erase(std::remove_if(neigh_seed.begin(), neigh_seed.end(),
        [&](int j){
          const float dz = std::fabs(cloud[j].z - S.z);
          if (dz > z_window_seed) {
            if (dumped < dump_max)
              cerr << "[local_grow]   drop(neigh zwin_seed) j="<< j
                   << " dz="<< fmtf(dz) << " > " << fmtf(z_window_seed) << "\n";
            ++dumped;
            return true;
          }
          return false;
        }), neigh_seed.end());
      cerr << "[local_grow]   neigh_seed after Z-window (±"<< fmtf(z_window_seed) << "): "
           << neigh_seed.size() << " (removed " << (before - neigh_seed.size()) << ")\n";
      global_rej.zwin_seed += (before - neigh_seed.size());
    }

    if (neigh_seed.size() < 2) {
      used_vertex[seed_idx] = 1;
      cerr << "[local_grow]   Insufficient neighbors (<2) after filtering. Seed skipped.\n";
      continue;
    }

    // Orden angular XY
    {
      const auto & Cc = cloud[seed_idx];
      std::sort(neigh_seed.begin(), neigh_seed.end(), [&](int a, int b){
        const float ax = cloud[a].x - Cc.x, ay = cloud[a].y - Cc.y;
        const float bx = cloud[b].x - Cc.x, by = cloud[b].y - Cc.y;
        return std::atan2(ay, ax) < std::atan2(by, bx);
      });
    }

    const size_t tri_off = triangles.size();
    tri_set.clear();
    edge_set.clear();

    // Contadores por componente
    RejectCounts comp_rej{};
    size_t comp_fan_attempts = 0, comp_fan_accept = 0;
    size_t comp_bfs_attempts = 0, comp_bfs_accept = 0;

    // 2) Abanico inicial (con radio XY relajado en lado vecino–vecino; ya lo teníamos)
    cerr << "[local_grow]   Fan: pairs=" << (neigh_seed.size() > 0 ? (neigh_seed.size()-1) : 0) << "\n";
    for (size_t t = 0; t + 1 < neigh_seed.size(); ++t) {
      const int j = neigh_seed[t];
      const int k = neigh_seed[t+1];
      comp_fan_attempts++; global_tri_fan_attempts++;

      bool dup=false;
      if (!precheck(seed_idx, j, k, Phase::FAN, comp_rej, dup)) {
        continue;
      }
      if (try_add_triangle(seed_idx, j, k, cloud, P, tri_set, edge_set, triangles)) {
        comp_fan_accept++; global_tri_fan_accept++;
      } else {
        if (!dup) { comp_rej.geom++; /* cerr << "[local_grow]     REJECT(geom)\n"; */ }
      }
    }
    // Cierre del abanico
    if (neigh_seed.size() >= 3) {
      const int j = neigh_seed.back();
      const int k = neigh_seed.front();
      comp_fan_attempts++; global_tri_fan_attempts++;

      bool dup=false;
      if (precheck(seed_idx, j, k, Phase::FAN, comp_rej, dup)) {
        if (try_add_triangle(seed_idx, j, k, cloud, P, tri_set, edge_set, triangles)) {
          comp_fan_accept++; global_tri_fan_accept++;
        } else {
          if (!dup) { comp_rej.geom++; }
        }
      }
    }

    cerr << "[local_grow]   Fan result: attempts=" << comp_fan_attempts
         << " accepted=" << comp_fan_accept
         << " tri_now=" << (triangles.size() - tri_off) << "\n";
    if (triangles.size() == tri_off) {
      used_vertex[seed_idx] = 1;
      cerr << "[local_grow]   No triangle from fan. Seed skipped.\n";
      // Acumula rechazos del componente en globales y sigue
      global_rej.edge_len += comp_rej.edge_len;
      global_rej.dz       += comp_rej.dz;
      global_rej.xy       += comp_rej.xy;
      global_rej.dup      += comp_rej.dup;
      global_rej.geom     += comp_rej.geom;
      global_rej.zwin_seed += comp_rej.zwin_seed;
      global_rej.zwin_bfs  += comp_rej.zwin_bfs;
      continue;
    }

    // Marca vértices usados
    for (size_t u = tri_off; u < triangles.size(); ++u) {
      used_vertex[triangles[u][0]] = 1;
      used_vertex[triangles[u][1]] = 1;
      used_vertex[triangles[u][2]] = 1;
    }

    // Inicializa frontera con aristas de los nuevos triángulos
    {
      size_t edges_queued = 0;
      for (size_t u = tri_off; u < triangles.size(); ++u) {
        const auto &t = triangles[u];
        frontier.emplace(EdgeKey(t[0], t[1])); edges_queued++;
        frontier.emplace(EdgeKey(t[1], t[2])); edges_queued++;
        frontier.emplace(EdgeKey(t[2], t[0])); edges_queued++;
      }
      cerr << "[local_grow]   Frontier initialized: " << edges_queued << " edges\n";
    }

    // 3) Expansión BFS (ahora con ventana Z respecto a la arista)
    size_t bfs_iters = 0;
    while (!frontier.empty()) {
      EdgeKey e = frontier.front(); frontier.pop();
      ++bfs_iters;

      const float z_mid = 0.5f * (cloud[e.a].z + cloud[e.b].z);
      const float z_window_up   = std::max(P.max_dz, 0.35f); // margen “hacia arriba”
      const float z_window_down = 0.10f;                     // margen “hacia abajo” (estricto)


      // Vecinos de e.a y e.b
      std::vector<int> neigh_a, neigh_b;
      {
        std::vector<int> inds; std::vector<float> dists;
        if (P.neighbor_radius > 0.0f) {
          if (kdtree.radiusSearch(cloud[e.a], P.neighbor_radius, inds, dists) > 0) {
            for (int id : inds) if (id != e.a) neigh_a.push_back(id);
          }
          inds.clear(); dists.clear();
          if (kdtree.radiusSearch(cloud[e.b], P.neighbor_radius, inds, dists) > 0) {
            for (int id : inds) if (id != e.b) neigh_b.push_back(id);
          }
        } else {
          const int K = std::max(8, P.k_neighbors);
          if (kdtree.nearestKSearch(cloud[e.a], K, inds, dists) > 0) {
            for (int id : inds) if (id != e.a) neigh_a.push_back(id);
          }
          inds.clear(); dists.clear();
          if (kdtree.nearestKSearch(cloud[e.b], K, inds, dists) > 0) {
            for (int id : inds) if (id != e.b) neigh_b.push_back(id);
          }
        }
      }

      for (int c : neigh_a) {
        if (c == e.a || c == e.b) continue;

        // Rápido: que c esté también cerca de e.b en XY (si hay radio)
        if (P.neighbor_radius > 0.0f) {
          const float dx = cloud[c].x - cloud[e.b].x;
          const float dy = cloud[c].y - cloud[e.b].y;
          if ((dx*dx + dy*dy) > P.neighbor_radius * P.neighbor_radius) continue;
        }

        const float dz = cloud[c].z - z_mid;
        if (dz < -z_window_down || dz > z_window_up) {
          ++global_rej.zwin_bfs;
          continue;
        }

        comp_bfs_attempts++; global_tri_bfs_attempts++;

        bool dup=false;
        if (!precheck(e.a, e.b, c, Phase::BFS, comp_rej, dup)) {
          continue;
        }
        if (try_add_triangle(e.a, e.b, c, cloud, P, tri_set, edge_set, triangles)) {
          comp_bfs_accept++; global_tri_bfs_accept++;
          frontier.emplace(EdgeKey(e.a, c));
          frontier.emplace(EdgeKey(c, e.b));
          used_vertex[e.a] = used_vertex[e.b] = used_vertex[c] = 1;
        } else {
          if (!dup) { comp_rej.geom++; }
        }
      }

      if (bfs_iters % 128 == 0) {
        cerr << "[local_grow]   BFS iters=" << bfs_iters
             << " tri_now=" << (triangles.size() - tri_off)
             << " frontier_size~" << frontier.size() << "\n";
      }
    }

    // Superficie final para esta semilla
    const size_t tri_cnt = triangles.size() - tri_off;
    if (tri_cnt > 0) {
      surf_tri_ranges.emplace_back(tri_off, tri_cnt);
    }

    // Acumular contadores globales
    global_rej.edge_len += comp_rej.edge_len;
    global_rej.dz       += comp_rej.dz;
    global_rej.xy       += comp_rej.xy;
    global_rej.dup      += comp_rej.dup;
    global_rej.geom     += comp_rej.geom;
    global_rej.zwin_seed += comp_rej.zwin_seed;
    global_rej.zwin_bfs  += comp_rej.zwin_bfs;

    cerr << "[local_grow]   Component done: triangles=" << tri_cnt
         << " (fan acc=" << comp_fan_accept << "/" << comp_fan_attempts
         << ", bfs acc=" << comp_bfs_accept << "/" << comp_bfs_attempts << ")\n";
    cerr << "[local_grow]   Rejects: edge=" << comp_rej.edge_len
         << " dz=" << comp_rej.dz
         << " xy=" << comp_rej.xy
         << " dup=" << comp_rej.dup
         << " geom=" << comp_rej.geom << "\n";
  }

  // Resumen global
  cerr << "\n[local_grow] ====== SUMMARY ======\n";
  cerr << "[local_grow] total triangles: " << triangles.size() << "\n";
  cerr << "[local_grow] components: " << surf_tri_ranges.size() << "\n";
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

  // 4) Construye NavMap + surfaces
  const std::string frame_id = "map";
  navmap_ros_interfaces::msg::NavMap msg_tmp;
  navmap::NavMap core;
  if (!build_navmap_from_mesh(cloud, triangles, frame_id, msg_tmp, &core)) {
    cerr << "[local_grow] build_navmap_from_mesh FAILED. RETURN empty.\n";
    return navmap::NavMap();
  }

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

  // Para probar la variante local:
  return from_points_local_grow(input_points, out_msg, params);
}

} // namespace navmap_ros
