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
#include <array>
#include <cmath>
#include <cassert>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include <numeric>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "navmap_ros_interfaces/msg/nav_map.hpp"
#include "navmap_ros_interfaces/msg/nav_map_layer.hpp"
#include "navmap_ros_interfaces/msg/nav_map_link.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/point_tests.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

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

namespace
{

// Layer data and metadata -> message.
void fill_layer_msg(
  const navmap::NavMap & nm, const std::string & name, const navmap::LayerViewBase & base,
  NavMapLayer & lmsg)
{
  lmsg.name = name;
  lmsg.type = static_cast<uint8_t>(base.type());
  lmsg.geometry_id = nm.geometry_id();
  switch (base.type()) {
    case navmap::LayerType::U8:
      lmsg.data_u8 = static_cast<const navmap::LayerView<uint8_t> &>(base).data();
      break;
    case navmap::LayerType::F32:
      lmsg.data_f32 = static_cast<const navmap::LayerView<float> &>(base).data();
      break;
    case navmap::LayerType::F64:
      lmsg.data_f64 = static_cast<const navmap::LayerView<double> &>(base).data();
      break;
    case navmap::LayerType::I16:
      lmsg.data_i16 = static_cast<const navmap::LayerView<int16_t> &>(base).data();
      break;
    default:
      throw std::runtime_error("to_msg(NavMapLayer): unsupported layer type");
  }
  if (const auto meta = nm.get_layer_meta(name)) {
    lmsg.description = meta->description;
    lmsg.unit = meta->unit;
    lmsg.has_unknown = meta->unknown.has_value();
    lmsg.unknown_value = meta->unknown.value_or(0.0);
  }
}

template<typename V>
void set_layer(navmap::NavMap & nm, const NavMapLayer & msg, const V & data)
{
  using T = typename V::value_type;
  auto dst = nm.add_layer<T>(msg.name, msg.description, msg.unit, T{});
  dst->data().assign(data.begin(), data.end());
  dst->mark_all_changed();
}

template<typename T, typename V>
void patch_layer(navmap::NavMap & nm, const NavMapLayer & msg, const V & data)
{
  auto view = std::dynamic_pointer_cast<navmap::LayerView<T>>(nm.layers.get(msg.name));
  if (!view) {
    throw std::runtime_error(
            "from_msg(NavMapLayer): sparse update of '" + msg.name +
          "', which is missing or of another type");
  }
  if (data.size() != msg.indices.size()) {
    throw std::runtime_error("from_msg(NavMapLayer): sparse update sizes differ");
  }
  auto & d = view->data();
  for (std::size_t k = 0; k < msg.indices.size(); ++k) {
    const auto c = msg.indices[k];
    if (c < d.size()) {
      d[c] = data[k];
      view->mark_changed(c);
    }
  }
}

// Message -> layer data and metadata (the layer is created or retyped as needed). A message
// naming another geometry, or a malformed sparse update, throws.
void apply_layer_msg(const NavMapLayer & msg, navmap::NavMap & nm)
{
  if (msg.geometry_id != 0 && nm.geometry_id() != msg.geometry_id && !nm.navcels.empty()) {
    throw std::runtime_error(
            "from_msg(NavMapLayer): layer '" + msg.name + "' belongs to another geometry");
  }
  if (!msg.indices.empty()) {
    switch (msg.type) {
      case NavMapLayer::U8: patch_layer<uint8_t>(nm, msg, msg.data_u8); break;
      case NavMapLayer::F32: patch_layer<float>(nm, msg, msg.data_f32); break;
      case NavMapLayer::F64: patch_layer<double>(nm, msg, msg.data_f64); break;
      case NavMapLayer::I16: patch_layer<int16_t>(nm, msg, msg.data_i16); break;
      default:
        throw std::runtime_error("from_msg(NavMapLayer): unsupported type value " +
          std::to_string(msg.type));
    }
    return;
  }
  if (auto existing = nm.layers.get(msg.name);
    existing && static_cast<uint8_t>(existing->type()) != msg.type)
  {
    nm.layers.remove(msg.name);
  }
  switch (msg.type) {
    case NavMapLayer::U8: set_layer(nm, msg, msg.data_u8); break;
    case NavMapLayer::F32: set_layer(nm, msg, msg.data_f32); break;
    case NavMapLayer::F64: set_layer(nm, msg, msg.data_f64); break;
    case NavMapLayer::I16: set_layer(nm, msg, msg.data_i16); break;
    default:
      throw std::runtime_error("from_msg(NavMapLayer): unsupported type value " +
        std::to_string(msg.type));
  }
  auto & meta = nm.layer_meta[msg.name];
  meta.description = msg.description;
  meta.unit = msg.unit;
  if (msg.has_unknown) {meta.unknown = msg.unknown_value;} else {meta.unknown.reset();}
}

}  // namespace

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
    fill_layer_msg(nm, lname, *base, lmsg);
    out.layers.push_back(std::move(lmsg));
  }

  out.geometry_id = nm.geometry_id();

  // links between non-adjacent NavCels
  out.links.reserve(nm.links().size());
  for (const auto & l : nm.links()) {
    navmap_ros_interfaces::msg::NavMapLink lm;
    lm.from_navcel = l.from;
    lm.to_navcel = l.to;
    lm.type = static_cast<uint8_t>(l.type);
    lm.length = l.length;
    lm.bidirectional = l.bidirectional;
    out.links.push_back(lm);
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
    apply_layer_msg(l, nm);
  }

  // links (out-of-range ones are dropped)
  for (const auto & l : msg.links) {
    nm.add_link(
      l.from_navcel, l.to_navcel, static_cast<navmap::LinkType>(l.type), l.length,
      l.bidirectional);
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
  fill_layer_msg(nm, layer_name, *base, msg);
  return msg;
}

navmap_ros_interfaces::msg::NavMapLayer to_msg(
  const navmap::NavMap & nm,
  const std::string & layer_name)
{
  return to_msg(nm, layer_name, std_msgs::msg::Header());
}

navmap_ros_interfaces::msg::NavMapLayer to_layer_update_msg(
  const navmap::NavMap & nm, const std::string & layer_name,
  const std_msgs::msg::Header & header, float max_sparse_fraction)
{
  auto base = nm.layers.get(layer_name);
  if (!base) {
    throw std::runtime_error("to_layer_update_msg: layer '" + layer_name + "' not found");
  }
  const auto & changed = base->changed();
  if (base->all_changed() || changed.empty() ||
    static_cast<float>(changed.size()) > max_sparse_fraction * static_cast<float>(base->size()))
  {
    return to_msg(nm, layer_name, header);  // full layer
  }
  NavMapLayer msg;
  msg.header = header;
  msg.name = layer_name;
  msg.type = static_cast<uint8_t>(base->type());
  msg.geometry_id = nm.geometry_id();
  if (const auto meta = nm.get_layer_meta(layer_name)) {
    msg.description = meta->description;
    msg.unit = meta->unit;
    msg.has_unknown = meta->unknown.has_value();
    msg.unknown_value = meta->unknown.value_or(0.0);
  }
  msg.indices.assign(changed.begin(), changed.end());
  auto gather = [&](const auto & data, auto & out) {
      out.reserve(changed.size());
      for (const auto c : changed) {
        out.push_back(data[c]);
      }
    };
  switch (base->type()) {
    case navmap::LayerType::U8:
      gather(static_cast<const navmap::LayerView<uint8_t> &>(*base).data(), msg.data_u8);
      break;
    case navmap::LayerType::F32:
      gather(static_cast<const navmap::LayerView<float> &>(*base).data(), msg.data_f32);
      break;
    case navmap::LayerType::F64:
      gather(static_cast<const navmap::LayerView<double> &>(*base).data(), msg.data_f64);
      break;
    case navmap::LayerType::I16:
      gather(static_cast<const navmap::LayerView<int16_t> &>(*base).data(), msg.data_i16);
      break;
  }
  return msg;
}

void from_msg(
  const navmap_ros_interfaces::msg::NavMapLayer & msg,
  navmap::NavMap & nm,
  std_msgs::msg::Header & header)
{
  header = msg.header;
  apply_layer_msg(msg, nm);
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

  // Per-NavCel occupancy layer (U8); NO_INFORMATION (from -1) is its unknown value
  auto occ = nm.layers.add_or_get<uint8_t>("occupancy", nm.navcels.size(), navmap::LayerType::U8);
  nm.set_layer_unknown("occupancy", NO_INFORMATION);
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
    layer.type = 1;  // F32
    layer.data_f32 = std::move(elev);
    out_msg.layers.push_back(std::move(layer));
  }

  if (out_core_opt) {
    *out_core_opt = navmap_ros::from_msg(out_msg);
  }
  return true;
}

using Triangle = Eigen::Vector3i;

// ----------------- Keep only largest surfaces -----------------

// Keeps only the triangles listed in some surface and the vertices they use.
static void compact_to_surfaces(navmap_ros_interfaces::msg::NavMap & msg)
{
  const std::size_t ntris = msg.navcels_v0.size();
  std::vector<uint32_t> tri_map(ntris, UINT32_MAX);
  uint32_t nt = 0;
  for (auto & s : msg.surfaces) {
    for (auto & cid : s.navcels) {
      if (cid >= ntris) {continue;}
      if (tri_map[cid] == UINT32_MAX) {tri_map[cid] = nt++;}
      cid = tri_map[cid];
    }
  }
  if (nt == ntris) {return;}

  const std::size_t nverts = msg.positions_x.size();
  std::vector<uint32_t> vmap(nverts, UINT32_MAX);
  uint32_t nv = 0;
  std::vector<uint32_t> v0(nt), v1(nt), v2(nt);
  auto remap_v = [&](uint32_t v) {
      if (vmap[v] == UINT32_MAX) {vmap[v] = nv++;}
      return vmap[v];
    };
  for (std::size_t t = 0; t < ntris; ++t) {
    if (tri_map[t] == UINT32_MAX) {continue;}
    v0[tri_map[t]] = remap_v(msg.navcels_v0[t]);
    v1[tri_map[t]] = remap_v(msg.navcels_v1[t]);
    v2[tri_map[t]] = remap_v(msg.navcels_v2[t]);
  }
  msg.navcels_v0.swap(v0);
  msg.navcels_v1.swap(v1);
  msg.navcels_v2.swap(v2);

  auto compact = [](auto & data, const std::vector<uint32_t> & map, uint32_t n) {
      if (data.size() != map.size()) {return;}
      std::remove_reference_t<decltype(data)> out(n);
      for (std::size_t k = 0; k < map.size(); ++k) {
        if (map[k] != UINT32_MAX) {out[map[k]] = data[k];}
      }
      data.swap(out);
    };
  compact(msg.positions_x, vmap, nv);
  compact(msg.positions_y, vmap, nv);
  compact(msg.positions_z, vmap, nv);
  if (msg.has_vertex_rgba) {
    compact(msg.colors_r, vmap, nv);
    compact(msg.colors_g, vmap, nv);
    compact(msg.colors_b, vmap, nv);
    compact(msg.colors_a, vmap, nv);
  }
  for (auto & layer : msg.layers) {
    compact(layer.data_u8, tri_map, nt);
    compact(layer.data_f32, tri_map, nt);
    compact(layer.data_f64, tri_map, nt);
  }
}

// Drops surfaces smaller than min_area (square meters) with their triangles and vertices.
static void drop_small_surfaces(navmap_ros_interfaces::msg::NavMap & msg, float min_area)
{
  if (!(min_area > 0.0f)) {return;}
  auto area_of = [&](uint32_t t) {
      auto p = [&](uint32_t v) {
          return Eigen::Vector3f(msg.positions_x[v], msg.positions_y[v], msg.positions_z[v]);
        };
      const Eigen::Vector3f a = p(msg.navcels_v0[t]);
      return 0.5f * (p(msg.navcels_v1[t]) - a).cross(p(msg.navcels_v2[t]) - a).norm();
    };
  std::vector<navmap_ros_interfaces::msg::NavMapSurface> kept;
  for (auto & s : msg.surfaces) {
    float area = 0.0f;
    for (uint32_t t : s.navcels) {
      area += area_of(t);
    }
    if (area >= min_area) {kept.push_back(std::move(s));}
  }
  msg.surfaces.swap(kept);
  compact_to_surfaces(msg);
}

static void keep_top_surfaces_by_size(navmap_ros_interfaces::msg::NavMap & msg, int max_surfaces)
{
  if (max_surfaces <= 0) {return;}
  if (msg.surfaces.size() <= static_cast<size_t>(max_surfaces)) {return;}

  std::vector<size_t> ids(msg.surfaces.size());
  std::iota(ids.begin(), ids.end(), 0);

  std::stable_sort(ids.begin(), ids.end(), [&](size_t a, size_t b){
      return msg.surfaces[a].navcels.size() > msg.surfaces[b].navcels.size();
  });

  std::vector<navmap_ros_interfaces::msg::NavMapSurface> kept;
  kept.reserve(static_cast<size_t>(max_surfaces));
  for (int i = 0; i < max_surfaces; ++i) {
    kept.push_back(std::move(msg.surfaces[ids[static_cast<size_t>(i)]]));
  }
  msg.surfaces.swap(kept);
  compact_to_surfaces(msg);
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

// ----------------- Multi-level grid builder (from_points) -----------------

namespace
{

// Height cluster of one XY cell: a vertex candidate of the multi-level grid.
struct GridNode
{
  int32_t i, j;
  float z;
  uint32_t support;  // points behind the node; 0 if interpolated
  float alt_z;       // height of a sparse upper face, if any (NaN otherwise)
  float z_min;       // lowest point of the level (its lower face, if any)
};

struct BinnedPoint
{
  uint64_t key;
  float z, x, y;  // x, y relative to the cell centre
};

float level_height_top(const BinnedPoint * pts, std::size_t n, float step);

// Height of one level at its cell centre. The level may hold several faces closer than the
// clearance (a slab seen from above and below): the support is its upper face, the top group
// of points above a gap. A top group too small to trust on its own (a sparse face, or a spike)
// is returned in alt_z, for the neighbours to decide.
float level_height(const BinnedPoint * pts, std::size_t n, float step, float & alt_z)
{
  alt_z = std::numeric_limits<float>::quiet_NaN();
  const float layer_gap = 0.5f * step;
  for (std::size_t k = n - 1; k >= 1; --k) {
    if (pts[k].z - pts[k - 1].z <= layer_gap) {continue;}
    if (n - k >= std::max<std::size_t>(3, n / 10)) {
      return level_height_top(pts + k, n - k, step);
    }
    if (std::isnan(alt_z)) {alt_z = pts[k + (n - k - 1) / 2].z;}  // median of the small group
  }
  return level_height_top(pts, n, step);
}

// Robust plane fit at the cell centre; a cluttered level keeps a high top (an obstacle).
float level_height_top(const BinnedPoint * pts, std::size_t n, float step)
{
  if (n < 3) {
    float z = 0.0f;
    for (std::size_t k = 0; k < n; ++k) {
      z += pts[k].z;
    }
    return z / static_cast<float>(n);
  }
  // Points come sorted by z: a vertical spread beyond a step that a plane cannot explain
  // means clutter (wall, bush, furniture), so keep the 90th percentile as an obstacle top.
  auto fit = [&](const std::vector<char> & use, Eigen::Vector3f & abc) {
      Eigen::Matrix3f A = Eigen::Matrix3f::Zero();
      Eigen::Vector3f b = Eigen::Vector3f::Zero();
      for (std::size_t k = 0; k < n; ++k) {
        if (!use[k]) {continue;}
        const Eigen::Vector3f r(pts[k].x, pts[k].y, 1.0f);
        A += r * r.transpose();
        b += r * pts[k].z;
      }
      if (std::fabs(A.determinant()) < 1e-9f) {return false;}
      abc = A.ldlt().solve(b);
      return abc.allFinite();
    };
  const float top90 = pts[std::min(n - 1, (n * 9) / 10)].z;
  std::vector<char> use(n, 1);
  Eigen::Vector3f abc;
  if (!fit(use, abc)) {
    // Collinear in XY (thin wall, scan line): no plane; tall means clutter.
    return pts[n - 1].z - pts[0].z > step ? top90 : pts[(n - 1) / 2].z;
  }
  // One trimming pass: drop residuals beyond 3 robust sigmas (spikes, outliers).
  std::vector<float> res(n);
  for (std::size_t k = 0; k < n; ++k) {
    res[k] = pts[k].z - (abc[0] * pts[k].x + abc[1] * pts[k].y + abc[2]);
  }
  std::vector<float> absr(n);
  for (std::size_t k = 0; k < n; ++k) {
    absr[k] = std::fabs(res[k]);
  }
  std::nth_element(absr.begin(), absr.begin() + n / 2, absr.end());
  const float thr = std::max(0.02f, 3.0f * 1.4826f * absr[n / 2]);
  std::size_t kept = 0;
  for (std::size_t k = 0; k < n; ++k) {
    use[k] = std::fabs(res[k]) <= thr;
    kept += use[k];
  }
  if (kept >= 3) {fit(use, abc);}

  // Clutter test on the inliers' spread around the plane (10th to 90th percentile).
  std::vector<float> in;
  in.reserve(n);
  for (std::size_t k = 0; k < n; ++k) {
    if (use[k]) {in.push_back(pts[k].z - (abc[0] * pts[k].x + abc[1] * pts[k].y + abc[2]));}
  }
  std::sort(in.begin(), in.end());
  const float spread = in[std::min(in.size() - 1, (in.size() * 9) / 10)] - in[in.size() / 10];
  return spread > step ? top90 : abc[2];
}

inline uint64_t cell_key(int32_t i, int32_t j)
{
  return (static_cast<uint64_t>(static_cast<uint32_t>(i)) << 32) | static_cast<uint32_t>(j);
}

// 8-neighbourhood; kOpposite[d] is the direction back.
constexpr int kDirs[8][2] = {{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
constexpr int kOpposite[8] = {4, 5, 6, 7, 0, 1, 2, 3};

inline int dir_index(int dx, int dy)
{
  for (int d = 0; d < 8; ++d) {
    if (kDirs[d][0] == dx && kDirs[d][1] == dy) {return d;}
  }
  return -1;
}

}  // namespace

navmap::NavMap from_points(
  const pcl::PointCloud<pcl::PointXYZ> & input_points,
  navmap_ros_interfaces::msg::NavMap & out_msg,
  BuildParams P)
{
  out_msg = navmap_ros_interfaces::msg::NavMap();
  const float res = P.resolution;
  if (!(res > 0.0f) || input_points.size() < 3) {
    return navmap::NavMap();
  }
  const float step = P.max_dz > 0.0f ? P.max_dz : 0.30f;
  // Points closer than this in z are the same surface (thickness, sampling, clutter).
  const float gap = std::max(step, P.min_clearance);
  const float tan_slope = std::tan(P.max_slope_deg * static_cast<float>(M_PI) / 180.0f);
  const float cos_slope = std::cos(P.max_slope_deg * static_cast<float>(M_PI) / 180.0f);

  // 1) Bin finite points by XY cell, sorted by (cell, z).
  float min_x = std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  for (const auto & p : input_points) {
    if (pcl::isFinite(p)) {min_x = std::min(min_x, p.x); min_y = std::min(min_y, p.y);}
  }
  if (!std::isfinite(min_x)) {
    return navmap::NavMap();
  }
  // Half-cell origin shift: samples on a res-spaced lattice fall on cell centres, not borders.
  min_x -= 0.5f * res;
  min_y -= 0.5f * res;
  std::vector<BinnedPoint> binned;
  binned.reserve(input_points.size());
  for (const auto & p : input_points) {
    if (!pcl::isFinite(p)) {continue;}
    const auto i = static_cast<int32_t>(std::floor((p.x - min_x) / res));
    const auto j = static_cast<int32_t>(std::floor((p.y - min_y) / res));
    binned.push_back(
      {cell_key(i, j), p.z, p.x - (min_x + (i + 0.5f) * res), p.y - (min_y + (j + 0.5f) * res)});
  }
  std::sort(
    binned.begin(), binned.end(), [](const BinnedPoint & a, const BinnedPoint & b) {
      return a.key != b.key ? a.key < b.key : a.z < b.z;
    });

  // 2) Split each cell into levels separated by more than the clearance gap.
  std::vector<GridNode> nodes;
  std::unordered_map<uint64_t, std::pair<uint32_t, uint32_t>> cells;  // key -> [first, count)
  cells.reserve(binned.size() / 4 + 1);
  for (std::size_t k = 0; k < binned.size(); ) {
    const uint64_t key = binned[k].key;
    const auto i = static_cast<int32_t>(key >> 32);
    const auto j = static_cast<int32_t>(key & 0xffffffffu);
    const auto first = static_cast<uint32_t>(nodes.size());
    std::size_t level_begin = k;
    for (++k; ; ++k) {
      const bool cell_end = k == binned.size() || binned[k].key != key;
      if (cell_end || binned[k].z - binned[k - 1].z > gap) {
        const std::size_t n = k - level_begin;
        float alt_z;
        const float z = level_height(&binned[level_begin], n, step, alt_z);
        nodes.push_back({i, j, z, static_cast<uint32_t>(n), alt_z, binned[level_begin].z});
        level_begin = k;
      }
      if (cell_end) {break;}
    }
    cells.emplace(key, std::make_pair(first, static_cast<uint32_t>(nodes.size()) - first));
  }

  // 2a) Sparse upper faces. A slab seen mostly from below leaves cells whose upper face got
  // few points (alt_z) or none, and their own height falls on the lower face.
  {
    const float tol = 0.5f * step;
    auto neighbours_with = [&](const GridNode & nd, auto && pred) {
        int votes = 0;
        for (const auto & d : kDirs) {
          const auto it = cells.find(cell_key(nd.i + d[0], nd.j + d[1]));
          if (it == cells.end()) {continue;}
          for (uint32_t m = it->second.first; m < it->second.first + it->second.second; ++m) {
            if (pred(nodes[m])) {++votes; break;}
          }
        }
        return votes;
      };
    // Pass 1: a sparse upper face is the support when a neighbour stands on it (its main
    // height) and two or more neighbours see it. Spikes over a floor never have a neighbour
    // whose main height is theirs.
    std::vector<std::pair<std::size_t, float>> lifted;
    for (std::size_t n = 0; n < nodes.size(); ++n) {
      const auto & nd = nodes[n];
      if (std::isnan(nd.alt_z)) {continue;}
      const int on_it = neighbours_with(
        nd, [&](const GridNode & nb) {return std::fabs(nb.z - nd.alt_z) <= tol;});
      const int seen = neighbours_with(
        nd, [&](const GridNode & nb) {
          return std::fabs(nb.z - nd.alt_z) <= tol ||
                 (!std::isnan(nb.alt_z) && std::fabs(nb.alt_z - nd.alt_z) <= tol);
        });
      if (on_it >= 1 && seen >= 2) {lifted.emplace_back(n, nd.alt_z);}
    }
    for (const auto & [n, z] : lifted) {
      nodes[n].z = z;
    }

    // Pass 2: a cell that only saw the lower face takes the upper face of the neighbours whose
    // lower face is at its height. It needs a level below in its column (where the sensor
    // was), so a pothole or a box never qualifies.
    std::vector<std::pair<std::size_t, float>> inferred;
    for (std::size_t n = 0; n < nodes.size(); ++n) {
      const auto & nd = nodes[n];
      bool level_below = false;
      const auto & own = cells.at(cell_key(nd.i, nd.j));
      for (uint32_t m = own.first; m < own.first + own.second; ++m) {
        level_below = level_below || nodes[m].z < nd.z_min - gap;
      }
      if (!level_below) {continue;}
      const int own_votes = neighbours_with(
        nd, [&](const GridNode & nb) {return std::fabs(nb.z - nd.z) <= tol;});
      std::vector<float> above;
      for (const auto & d : kDirs) {
        const auto it = cells.find(cell_key(nd.i + d[0], nd.j + d[1]));
        if (it == cells.end()) {continue;}
        for (uint32_t m = it->second.first; m < it->second.first + it->second.second; ++m) {
          const auto & nb = nodes[m];
          if (std::fabs(nb.z_min - nd.z) <= tol && nb.z > nd.z + tol && nb.z - nd.z <= gap) {
            above.push_back(nb.z);
            break;
          }
        }
      }
      if (above.size() >= 3 && static_cast<int>(above.size()) > own_votes) {
        std::nth_element(above.begin(), above.begin() + above.size() / 2, above.end());
        inferred.emplace_back(n, above[above.size() / 2]);
      }
    }
    for (const auto & [n, z] : inferred) {
      nodes[n].z = z;
      nodes[n].support = 0;  // height inferred, not observed
    }
  }

  // 2b) Fill small enclosed holes (cells without any point) from the levels around them.
  const auto max_hole_cells = static_cast<std::size_t>(std::floor(P.max_hole_area / (res * res)));
  if (max_hole_cells > 0) {
    std::unordered_set<uint64_t> visited;
    std::vector<std::pair<uint64_t, std::vector<float>>> filled;
    std::vector<uint64_t> seeds;
    for (const auto & kv : cells) {
      const auto i = static_cast<int32_t>(kv.first >> 32);
      const auto j = static_cast<int32_t>(kv.first & 0xffffffffu);
      for (int d = 0; d < 8; d += 2) {
        seeds.push_back(cell_key(i + kDirs[d][0], j + kDirs[d][1]));
      }
    }
    std::sort(seeds.begin(), seeds.end());
    for (const uint64_t seed : seeds) {
      if (cells.count(seed) || visited.count(seed)) {continue;}
      // Flood the empty region (4-connected); give up once it is larger than a hole.
      std::vector<uint64_t> hole{seed}, stack{seed};
      visited.insert(seed);
      bool too_big = false;
      while (!stack.empty() && !too_big) {
        const uint64_t c = stack.back();
        stack.pop_back();
        const auto ci = static_cast<int32_t>(c >> 32);
        const auto cj = static_cast<int32_t>(c & 0xffffffffu);
        for (int d = 0; d < 8; d += 2) {
          const uint64_t nk = cell_key(ci + kDirs[d][0], cj + kDirs[d][1]);
          if (cells.count(nk) || visited.count(nk)) {continue;}
          visited.insert(nk);
          hole.push_back(nk);
          stack.push_back(nk);
          if (hole.size() > max_hole_cells) {too_big = true; break;}
        }
      }
      if (too_big) {continue;}

      // Border cells (8-neighbours with data) and their levels.
      std::unordered_set<uint64_t> in_hole(hole.begin(), hole.end());
      std::unordered_set<uint64_t> border;
      for (const uint64_t c : hole) {
        const auto ci = static_cast<int32_t>(c >> 32);
        const auto cj = static_cast<int32_t>(c & 0xffffffffu);
        for (const auto & dd : kDirs) {
          const uint64_t nk = cell_key(ci + dd[0], cj + dd[1]);
          if (!in_hole.count(nk) && cells.count(nk)) {border.insert(nk);}
        }
      }
      std::vector<uint32_t> border_nodes;
      for (const uint64_t b : border) {
        const auto & r = cells.at(b);
        for (uint32_t n = r.first; n < r.first + r.second; ++n) {
          if (nodes[n].support > 0) {border_nodes.push_back(n);}
        }
      }
      std::sort(
        border_nodes.begin(), border_nodes.end(),
        [&](uint32_t a, uint32_t b) {return nodes[a].z < nodes[b].z;});

      std::vector<std::vector<float>> zs(hole.size());
      for (std::size_t g = 0; g < border_nodes.size(); ) {
        // One level: border nodes whose heights chain within the clearance gap.
        std::size_t e = g + 1;
        while (e < border_nodes.size() &&
          nodes[border_nodes[e]].z - nodes[border_nodes[e - 1]].z <= gap)
        {
          ++e;
        }
        std::unordered_set<uint64_t> level_cells;
        for (std::size_t q = g; q < e; ++q) {
          level_cells.insert(cell_key(nodes[border_nodes[q]].i, nodes[border_nodes[q]].j));
        }
        // Only levels present around most of the hole (not a single wall or ledge).
        if (level_cells.size() * 4 >= border.size() * 3) {
          for (std::size_t h = 0; h < hole.size(); ++h) {
            const auto hi = static_cast<int32_t>(hole[h] >> 32);
            const auto hj = static_cast<int32_t>(hole[h] & 0xffffffffu);
            double wsum = 0.0, zsum = 0.0;
            for (std::size_t q = g; q < e; ++q) {
              const auto & nd = nodes[border_nodes[q]];
              const double d2 = static_cast<double>(nd.i - hi) * (nd.i - hi) +
                static_cast<double>(nd.j - hj) * (nd.j - hj);
              wsum += 1.0 / d2;
              zsum += nd.z / d2;
            }
            zs[h].push_back(static_cast<float>(zsum / wsum));
          }
        }
        g = e;
      }
      for (std::size_t h = 0; h < hole.size(); ++h) {
        if (!zs[h].empty()) {filled.emplace_back(hole[h], std::move(zs[h]));}
      }
    }
    for (auto & [key, levels] : filled) {
      const auto first = static_cast<uint32_t>(nodes.size());
      for (const float z : levels) {
        nodes.push_back(
          {static_cast<int32_t>(key >> 32), static_cast<int32_t>(key & 0xffffffffu), z, 0,
            std::numeric_limits<float>::quiet_NaN(), z});
      }
      cells.emplace(key, std::make_pair(first, static_cast<uint32_t>(levels.size())));
    }
  }

  // 3) Link nodes of neighbouring cells: closest height within step + slope, mutual.
  auto partner_in = [&](const GridNode & a, int d) -> int64_t {
      const auto it = cells.find(cell_key(a.i + kDirs[d][0], a.j + kDirs[d][1]));
      if (it == cells.end()) {return -1;}
      const float dist = res * ((kDirs[d][0] != 0 && kDirs[d][1] != 0) ? std::sqrt(2.0f) : 1.0f);
      const float max_dz = step + tan_slope * dist;
      int64_t best = -1;
      float best_dz = std::numeric_limits<float>::infinity();
      for (uint32_t n = it->second.first; n < it->second.first + it->second.second; ++n) {
        const float dz = std::fabs(nodes[n].z - a.z);
        if (dz <= max_dz && dz < best_dz) {best_dz = dz; best = n;}
      }
      return best;
    };
  std::vector<std::array<int64_t, 8>> link(nodes.size());
  for (std::size_t n = 0; n < nodes.size(); ++n) {
    for (int d = 0; d < 8; ++d) {
      link[n][d] = partner_in(nodes[n], d);
    }
  }
  for (std::size_t n = 0; n < nodes.size(); ++n) {
    for (int d = 0; d < 8; ++d) {
      const int64_t m = link[n][d];
      if (m >= 0 && link[m][kOpposite[d]] != static_cast<int64_t>(n)) {link[n][d] = -1;}
    }
  }
  auto linked = [&](int64_t a, int64_t b, int dx, int dy) {
      return a >= 0 && b >= 0 && link[a][dir_index(dx, dy)] == b;
    };

  // 4) Triangulate each quad: per connected sheet, the triangles of one diagonal.
  std::vector<uint64_t> quads;
  quads.reserve(cells.size() * 4);
  for (const auto & kv : cells) {
    const auto i = static_cast<int32_t>(kv.first >> 32);
    const auto j = static_cast<int32_t>(kv.first & 0xffffffffu);
    for (int di = -1; di <= 0; ++di) {
      for (int dj = -1; dj <= 0; ++dj) {
        quads.push_back(cell_key(i + di, j + dj));
      }
    }
  }
  std::sort(quads.begin(), quads.end());
  quads.erase(std::unique(quads.begin(), quads.end()), quads.end());

  auto vtx = [&](int64_t n) {
      return Eigen::Vector3f(
        min_x + (nodes[n].i + 0.5f) * res, min_y + (nodes[n].j + 0.5f) * res, nodes[n].z);
    };
  auto good_triangle = [&](int64_t a, int64_t b, int64_t c) {
      const Eigen::Vector3f n = (vtx(b) - vtx(a)).cross(vtx(c) - vtx(a));
      const float nn = n.norm();
      return 0.5f * nn >= P.min_area && nn > 0.0f && n.z() / nn >= cos_slope;
    };

  std::vector<std::array<int64_t, 3>> tris;
  tris.reserve(nodes.size() * 2);
  // Corners: 0=(i,j) 1=(i+1,j) 2=(i+1,j+1) 3=(i,j+1).
  constexpr int kCorner[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
  for (const uint64_t q : quads) {
    const auto qi = static_cast<int32_t>(q >> 32);
    const auto qj = static_cast<int32_t>(q & 0xffffffffu);

    std::vector<std::pair<int, int64_t>> members;  // (corner, node)
    for (int c = 0; c < 4; ++c) {
      const auto it = cells.find(cell_key(qi + kCorner[c][0], qj + kCorner[c][1]));
      if (it == cells.end()) {continue;}
      for (uint32_t n = it->second.first; n < it->second.first + it->second.second; ++n) {
        members.emplace_back(c, n);
      }
    }
    if (members.size() < 3) {continue;}

    std::vector<char> seen(members.size(), 0);
    for (std::size_t s = 0; s < members.size(); ++s) {
      if (seen[s]) {continue;}
      // Sheet: nodes of this quad connected through links.
      std::array<int64_t, 4> corner{-1, -1, -1, -1};
      bool ambiguous = false;
      std::vector<std::size_t> stack{s};
      seen[s] = 1;
      while (!stack.empty()) {
        const auto [c, n] = members[stack.back()];
        stack.pop_back();
        if (corner[c] >= 0 && corner[c] != n) {ambiguous = true;}
        corner[c] = n;
        for (std::size_t t = 0; t < members.size(); ++t) {
          const auto [c2, n2] = members[t];
          if (seen[t] || c2 == c) {continue;}
          if (linked(n, n2, kCorner[c2][0] - kCorner[c][0], kCorner[c2][1] - kCorner[c][1])) {
            seen[t] = 1;
            stack.push_back(t);
          }
        }
      }
      if (ambiguous) {continue;}

      const int64_t v00 = corner[0], v10 = corner[1], v11 = corner[2], v01 = corner[3];
      auto ok = [&](int64_t a, int64_t b, int64_t c) {
          if (a < 0 || b < 0 || c < 0) {return false;}
          auto lk = [&](int64_t x, int64_t y) {
              const auto & nx = nodes[x];
              const auto & ny = nodes[y];
              return linked(x, y, ny.i - nx.i, ny.j - nx.j);
            };
          return lk(a, b) && lk(b, c) && lk(c, a) && good_triangle(a, b, c);
        };
      const bool a1 = ok(v00, v10, v11), a2 = ok(v00, v11, v01);
      const bool b1 = ok(v00, v10, v01), b2 = ok(v10, v11, v01);
      const int na = a1 + a2, nb = b1 + b2;
      bool use_a = na >= nb;
      if (na == 2 && nb == 2) {
        // Both diagonals possible: the one with the smaller height change.
        use_a = std::fabs(nodes[v00].z - nodes[v11].z) <= std::fabs(nodes[v10].z - nodes[v01].z);
      }
      if (use_a) {
        if (a1) {tris.push_back({v00, v10, v11});}
        if (a2) {tris.push_back({v00, v11, v01});}
      } else {
        if (b1) {tris.push_back({v00, v10, v01});}
        if (b2) {tris.push_back({v10, v11, v01});}
      }
    }
  }
  if (tris.empty()) {
    return navmap::NavMap();
  }

  // 5) Vertices: only nodes used by some triangle.
  std::vector<int64_t> vid(nodes.size(), -1);
  pcl::PointCloud<pcl::PointXYZ> verts;
  std::vector<Triangle> triangles;
  triangles.reserve(tris.size());
  for (const auto & t : tris) {
    Triangle out;
    for (int k = 0; k < 3; ++k) {
      if (vid[t[k]] < 0) {
        vid[t[k]] = static_cast<int64_t>(verts.size());
        const Eigen::Vector3f p = vtx(t[k]);
        verts.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
      }
      out[k] = static_cast<int>(vid[t[k]]);
    }
    triangles.push_back(out);
  }

  const std::string frame_id = "map";
  navmap_ros_interfaces::msg::NavMap msg_tmp;
  if (!build_navmap_from_mesh(verts, triangles, frame_id, msg_tmp, nullptr)) {
    return navmap::NavMap();
  }
  {
    // Points behind each triangle (its weakest vertex); 0 = interpolated over a hole.
    navmap_ros_interfaces::msg::NavMapLayer support;
    support.name = "support";
    support.type = navmap_ros_interfaces::msg::NavMapLayer::U8;
    support.data_u8.reserve(tris.size());
    for (const auto & t : tris) {
      const uint32_t m = std::min({nodes[t[0]].support, nodes[t[1]].support, nodes[t[2]].support});
      support.data_u8.push_back(static_cast<uint8_t>(std::min<uint32_t>(m, 255u)));
    }
    msg_tmp.layers.push_back(std::move(support));
  }
  rebuild_surfaces_by_connectivity(msg_tmp);
  drop_small_surfaces(msg_tmp, P.min_surface_area);
  keep_top_surfaces_by_size(msg_tmp, P.max_surfaces);
  if (msg_tmp.navcels_v0.empty()) {
    return navmap::NavMap();
  }

  out_msg = std::move(msg_tmp);
  return from_msg(out_msg);
}

// ----------------- Regular-grid builder (deterministic, gap-free) -----------------

navmap::NavMap from_regular_grid(
  const pcl::PointCloud<pcl::PointXYZ> & grid_points,
  navmap_ros_interfaces::msg::NavMap & out_msg,
  BuildParams P)
{
  out_msg = navmap_ros_interfaces::msg::NavMap();

  const uint32_t W = grid_points.width;
  const uint32_t H = grid_points.height;
  if (H <= 1 || W < 2 || H < 2 || grid_points.size() != static_cast<size_t>(W) * H) {
    return navmap::NavMap();
  }

  auto v_id = [W](uint32_t i, uint32_t j) {return static_cast<int>(j * W + i);};
  const float cos_max_slope =
    std::cos(P.max_slope_deg * static_cast<float>(M_PI) / 180.0f);

  std::vector<Triangle> triangles;
  triangles.reserve(static_cast<size_t>(2) * (W - 1) * (H - 1));

  // One grid cell -> up to two triangles, split along the same diagonal
  // from_occupancy_grid uses. No neighbor search: connectivity is already
  // fully known from the grid indices, so the only reason to skip a
  // triangle is a non-finite vertex or a real slope violation -- never a
  // search-radius/angle heuristic, so this cannot leave a meshing-artifact
  // hole on an otherwise-navigable, evenly-sampled surface.
  auto try_cell_triangle = [&](int i0, int i1, int i2) {
      const auto & A = grid_points[i0];
      const auto & B = grid_points[i1];
      const auto & C = grid_points[i2];
      if (!pcl::isFinite(A) || !pcl::isFinite(B) || !pcl::isFinite(C)) {return;}

      Eigen::Vector3f a(A.x, A.y, A.z), b(B.x, B.y, B.z), c(C.x, C.y, C.z);
      Eigen::Vector3f n = (b - a).cross(c - a);
      const float nn = n.norm();
      if (nn < 1e-9f) {return;}
      n /= nn;
      if (n.dot(Eigen::Vector3f::UnitZ()) < cos_max_slope) {return;}

      // Canonical orientation (normal facing +Z), matching try_add_triangle.
      if (n.dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
        triangles.emplace_back(i0, i2, i1);
      } else {
        triangles.emplace_back(i0, i1, i2);
      }
    };

  for (uint32_t j = 0; j + 1 < H; ++j) {
    for (uint32_t i = 0; i + 1 < W; ++i) {
      const int id00 = v_id(i, j);
      const int id10 = v_id(i + 1, j);
      const int id11 = v_id(i + 1, j + 1);
      const int id01 = v_id(i, j + 1);
      try_cell_triangle(id00, id10, id11);
      try_cell_triangle(id00, id11, id01);
    }
  }

  if (triangles.empty()) {
    return navmap::NavMap();
  }

  const std::string frame_id = "map";
  navmap_ros_interfaces::msg::NavMap msg_tmp;
  navmap::NavMap core;
  if (!build_navmap_from_mesh(grid_points, triangles, frame_id, msg_tmp, &core)) {
    return navmap::NavMap();
  }

  // Single surface covering every accepted triangle; rebuild_surfaces_by_
  // connectivity then splits it by real adjacency, so terrain separated by
  // a too-steep band ends up as distinct surfaces, same as from_points.
  msg_tmp.surfaces.clear();
  {
    navmap_ros_interfaces::msg::NavMapSurface s;
    s.frame_id = frame_id;
    s.navcels.resize(triangles.size());
    for (size_t k = 0; k < triangles.size(); ++k) {
      s.navcels[k] = static_cast<uint32_t>(k);
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

}  // namespace navmap_ros
