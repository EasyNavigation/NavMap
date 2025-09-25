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

  // Generic fallback: sample each cell center with closest_triangle()
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

      // Use closest_triangle (const) instead of locate_navcel (non-const)
      if (nm.closest_triangle({cx, cy, static_cast<float>(g.info.origin.position.z)},
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

  // 3) Per-navcel "elevation" layer (float32): mean Z of triangle vertices
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

  // 4) Convert to core structure if requested
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

// Keys to avoid duplicates
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

  // Max edge length
  auto dAB = dist3(A, B);
  auto dBC = dist3(B, C);
  auto dCA = dist3(C, A);
  if (dAB > P.max_edge_len || dBC > P.max_edge_len || dCA > P.max_edge_len) {return false;}

  Eigen::Vector3f a(A.x, A.y, A.z);
  Eigen::Vector3f b(B.x, B.y, B.z);
  Eigen::Vector3f c(C.x, C.y, C.z);

  // Min area
  if (tri_area(a, b, c) < P.min_area) {return false;}

  // Max slope
  float slope = tri_slope_deg(a, b, c);
  if (slope > P.max_slope_deg) {return false;}

  // Min angle at each vertex
  float angA, angB, angC;
  triangle_angles_deg(a, b, c, angA, angB, angC);
  if (angA < P.min_angle_deg || angB < P.min_angle_deg || angC < P.min_angle_deg) {
    return false;
  }

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

  // Emit one point per voxel
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
      if (dist3(V, Pn) <= P.max_edge_len) {
        candidates.emplace_back(idx, Eigen::Vector3f(Pn.x, Pn.y, Pn.z));
      }
    }
    if (candidates.size() < 2) {continue;}

    // 3) Angular order on the local tangent plane at v
    Eigen::Vector3f vpos(V.x, V.y, V.z);
    std::vector<int> ordered;
    sort_neighbors_angular(vpos, candidates, ordered);

    // 4) Try fan triangles from consecutive pairs
    //    Optionally close the fan by connecting last with first.
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
    int j0 = ordered.front(), k0 = ordered.back();
    try_add_triangle(v, k0, j0, cloud, P, tri_set, edge_set, tris);
  }

  return tris;
}


navmap::NavMap from_points(
  const pcl::PointCloud<pcl::PointXYZ> & input_points,
  navmap_ros_interfaces::msg::NavMap & out_msg,
  BuildParams params)
{
  // --- Overview ------------------------------------------------------------
  // 1) Optional voxel downsampling in XY (averaging XYZ).
  // 2) 2D Delaunay triangulation in the XY plane (no crossing edges by construction).
  // 3) Lift triangles back to 3D and filter by:
  //    - area bounds,
  //    - maximum edge length,
  //    - maximum vertical discontinuity per edge (max_dz),
  //    - maximum slope w.r.t. +Z (max_slope_deg).
  // 4) Emit a single-surface NavMap and mirror it into out_msg.
  // -------------------------------------------------------------------------

  using std::vector;

  if (input_points.empty() || input_points.size() < 3) {
    out_msg = navmap_ros_interfaces::msg::NavMap();
    return navmap::NavMap();
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (params.resolution > 0.0f) {
    cloud = downsample_voxelize_avgXYZ(input_points, params.resolution);
  } else {
    cloud = input_points;
  }
  if (cloud.size() < 3) {
    out_msg = navmap_ros_interfaces::msg::NavMap();
    return navmap::NavMap();
  }

  struct Pt2 { double x, y; int idx3d; };
  struct Tri2 { int a, b, c; };

  auto orient2d = [](const Pt2 & a, const Pt2 & b, const Pt2 & c) -> double {
      return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    };

  auto in_circumcircle = [&](const Pt2 & p,
    const Pt2 & A, const Pt2 & B, const Pt2 & C) -> bool
    {
      const double ax = A.x - p.x, ay = A.y - p.y;
      const double bx = B.x - p.x, by = B.y - p.y;
      const double cx = C.x - p.x, cy = C.y - p.y;
      const double det = (ax * ax + ay * ay) * (bx * cy - by * cx) -
        (bx * bx + by * by) * (ax * cy - ay * cx) +
        (cx * cx + cy * cy) * (ax * by - ay * bx);
      const double o = orient2d(A, B, C);
      return (o > 0.0) ? (det > 0.0) : (det < 0.0);
    };

  vector<Pt2> pts2; pts2.reserve(cloud.size());
  for (int i = 0; i < static_cast<int>(cloud.size()); ++i) {
    pts2.push_back({static_cast<double>(cloud[i].x),
        static_cast<double>(cloud[i].y), i});
  }

  double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
  for (const auto & p : pts2) {
    if (p.x < minx) {minx = p.x;}
    if (p.y < miny) {miny = p.y;}
    if (p.x > maxx) {maxx = p.x;}
    if (p.y > maxy) {maxy = p.y;}
  }
  const double dx = maxx - minx, dy = maxy - miny, d = std::max(dx, dy);
  Pt2 S1{minx - 10 * d, miny - d, -1};
  Pt2 S2{minx + 0.5 * d, maxy + 10 * d, -2};
  Pt2 S3{maxx + 10 * d, miny - d, -3};

  vector<Pt2> vs = pts2;
  const int iS1 = static_cast<int>(vs.size()); vs.push_back(S1);
  const int iS2 = static_cast<int>(vs.size()); vs.push_back(S2);
  const int iS3 = static_cast<int>(vs.size()); vs.push_back(S3);

  vector<Tri2> tris;
  if (orient2d(vs[iS1], vs[iS2], vs[iS3]) <= 0.0) {
    std::swap(vs[iS2], vs[iS3]);
  }
  tris.push_back({iS1, iS2, iS3});

  // Incremental Bowyer–Watson
  for (int pi = 0; pi < static_cast<int>(pts2.size()); ++pi) {
    const Pt2 p = vs[pi];

    // 2.1) Collect "bad" triangles (circumcircle contains p)
    vector<int> bad; bad.reserve(tris.size());
    for (int ti = 0; ti < static_cast<int>(tris.size()); ++ti) {
      const auto & t = tris[ti];
      if (in_circumcircle(p, vs[t.a], vs[t.b], vs[t.c])) {
        bad.push_back(ti);
      }
    }

    // 2.2) Boundary of the polygonal cavity: edges touched exactly once
    struct EdgeKey
    {
      int u, v;
      bool operator==(const EdgeKey & o) const noexcept
      {
        return u == o.u && v == o.v;
      }
    };
    struct EdgeKeyHash
    {
      std::size_t operator()(const EdgeKey & e) const noexcept
      {
        return (static_cast<std::size_t>(e.u) << 32) ^ static_cast<std::size_t>(e.v);
      }
    };

    std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_count;
    edge_count.reserve(bad.size() * 3);

    auto add_edge = [&](int u, int v) {
      // canonicalize (min, max) so opposite directions collide
        if (u > v) {std::swap(u, v);}
        EdgeKey e{u, v};
        ++edge_count[e];
      };

    for (int ti : bad) {
      const auto & t = tris[ti];
      add_edge(t.a, t.b);
      add_edge(t.b, t.c);
      add_edge(t.c, t.a);
    }

    // 2.3) Remove bad triangles
    vector<Tri2> kept; kept.reserve(tris.size());
    vector<char> removed(tris.size(), 0);
    for (int idx : bad) {
      removed[idx] = 1;
    }
    for (int ti = 0; ti < static_cast<int>(tris.size()); ++ti) {
      if (!removed[ti]) {kept.push_back(tris[ti]);}
    }
    tris.swap(kept);

    // 2.4) Retriangulate the cavity with p
    for (const auto & kv : edge_count) {
      if (kv.second != 1) {continue;} // interior edges appear twice
      int u = kv.first.u, v = kv.first.v;
      // enforce CCW for (u, v, p)
      if (orient2d(vs[u], vs[v], p) <= 0.0) {std::swap(u, v);}
      tris.push_back({u, v, pi});
    }
  }

  // 2.5) Discard triangles touching the super-triangle
  vector<Tri2> final_tris; final_tris.reserve(tris.size());
  for (const auto & t : tris) {
    if (t.a >= static_cast<int>(pts2.size()) ||
      t.b >= static_cast<int>(pts2.size()) ||
      t.c >= static_cast<int>(pts2.size()))
    {
      continue;
    }
    final_tris.push_back(t);
  }

  // ---- 3) Lift to 3D and filter ------------------------------------------
  auto tri_area3D = [](const Eigen::Vector3f & A,
    const Eigen::Vector3f & B,
    const Eigen::Vector3f & C) -> float {
      return 0.5f * ((B - A).cross(C - A)).norm();
    };
  auto tri_normal = [](const Eigen::Vector3f & A,
    const Eigen::Vector3f & B,
    const Eigen::Vector3f & C) -> Eigen::Vector3f {
      Eigen::Vector3f n = (B - A).cross(C - A);
      const float L = n.norm();
      return (L > 1e-12f) ? (n / L) : Eigen::Vector3f(0.f, 0.f, 1.f);
    };
  auto edge_len3D = [&](int i, int j) -> float {
      const auto & a = cloud[i]; const auto & b = cloud[j];
      const float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
      return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

  const float cos_max_slope =
    std::cos(params.max_slope_deg * static_cast<float>(M_PI) / 180.0f);
  const float min_area = std::max(params.min_area, 1e-9f);
  const float max_edge =
    (params.max_edge_len > 0.0f) ?
    params.max_edge_len :
    std::numeric_limits<float>::infinity();
  const float max_dz =
    (params.max_dz > 0.0f) ?
    params.max_dz :
    std::numeric_limits<float>::infinity();

  vector<Eigen::Vector3i> triangles;
  triangles.reserve(final_tris.size());

  // Debug counters
  size_t dropped_area = 0, dropped_edge = 0, dropped_dz = 0, dropped_slope = 0;

  for (const auto & t : final_tris) {
    const int ia = vs[t.a].idx3d;
    const int ib = vs[t.b].idx3d;
    const int ic = vs[t.c].idx3d;

    const Eigen::Vector3f A(cloud[ia].x, cloud[ia].y, cloud[ia].z);
    const Eigen::Vector3f B(cloud[ib].x, cloud[ib].y, cloud[ib].z);
    const Eigen::Vector3f C(cloud[ic].x, cloud[ic].y, cloud[ic].z);

    const float area = tri_area3D(A, B, C);
    if (area < min_area) {++dropped_area; continue;}

    const float lAB = edge_len3D(ia, ib);
    const float lBC = edge_len3D(ib, ic);
    const float lCA = edge_len3D(ic, ia);
    if (lAB > max_edge || lBC > max_edge || lCA > max_edge) {++dropped_edge; continue;}

    const float dzAB = std::fabs(A.z() - B.z());
    const float dzBC = std::fabs(B.z() - C.z());
    const float dzCA = std::fabs(C.z() - A.z());
    if (std::max({dzAB, dzBC, dzCA}) > max_dz) {++dropped_dz; continue;}

    const Eigen::Vector3f n = tri_normal(A, B, C);
    if (n.dot(Eigen::Vector3f::UnitZ()) < cos_max_slope) {++dropped_slope; continue;}

    triangles.emplace_back(ia, ib, ic);
  }

  // ---- 4) Emit message + core --------------------------------------------
  navmap::NavMap core;
  build_navmap_from_mesh(cloud, triangles, /*frame_id*/ "map", out_msg, &core);

  // ---- Debug report ------------------------------------------------------
  // std::cerr << "[from_points] Candidate tris: " << final_tris.size()
  //           << " | Accepted: " << triangles.size()
  //           << " | Dropped (area=" << dropped_area
  //           << ", edge=" << dropped_edge
  //           << ", dz=" << dropped_dz
  //           << ", slope=" << dropped_slope
  //           << ")\n";

  return core;
}


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
