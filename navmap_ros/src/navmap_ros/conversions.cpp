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

#include "navmap_ros/conversions.hpp"

#include <algorithm>
#include <cmath>
#include <cassert>
#include <unordered_map>

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>

#include <navmap_core/Geometry.hpp>
#include <navmap_ros_interfaces/msg/nav_map_layer.hpp>
#include <navmap_ros_interfaces/msg/nav_map_surface.hpp>

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

// Given grid dims (W,H) and pattern=0, the two triangles indices for cell (i,j):
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

  // 5) Per-NavCel layer "occupancy" (U8), two tris per cell with same value
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
  // We infer W,H, res and origin from the surface AABB & vertex lattice.
  // Assumptions: single flat Z plane, regular spacing in X,Y, raster tri order.
  if (nm.surfaces.size() == 1) {
    // Heuristic: infer W,H by counting unique X and Y coordinates.
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

  // (Optional) Vertex colors -> out_msg.has_vertex_rgba = true; ... (r,g,b,a)
  // (Optional) Surfaces -> you can group navcels by surface later

  // 4) Convert a kernel if requested
  if (out_core_opt) {
    *out_core_opt = navmap_ros::from_msg(out_msg);
  }
  return true;
}

} // namespace navmap_ros
