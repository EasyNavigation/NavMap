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

#include "navmap_tools/cloud_to_navmap.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "navmap_ros/conversions.hpp"
#include "navmap_tools/floor_holes.hpp"
#include "navmap_tools/pcd_stream.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace navmap_tools
{

namespace
{
std::uint64_t cell_key(std::int64_t i, std::int64_t j)
{
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(i)) << 32) |
         static_cast<std::uint32_t>(j);
}

std::string sformat(const char * fmt, double a = 0, double b = 0, double c = 0, double d = 0)
{
  char buf[512];
  std::snprintf(buf, sizeof(buf), fmt, a, b, c, d);
  return buf;
}

struct CellCount
{
  std::uint32_t raw{0}, kept{0};
};
}  // namespace

std::string options_the_grid_mesher_ignores(const Options & opt)
{
  const Options d;
  std::string out;
  auto add = [&out](bool set, const char * name) {
      if (set) {out += (out.empty() ? "" : ", ") + std::string(name);}
    };
  add(opt.crop, "crop");
  add(opt.recenter, "recenter");
  add(std::isfinite(opt.zmin) || std::isfinite(opt.zmax), "zmin/zmax");
  add(opt.fill_holes > 0.0f, "fill_holes");
  add(opt.voxel > 0.0f, "voxel");
  add(opt.min_voxel_points > 1, "min_voxel_points");
  add(opt.max_dz != d.max_dz, "max_dz");
  add(opt.min_clearance != d.min_clearance, "min_clearance");
  add(opt.max_hole_area != d.max_hole_area, "max_hole_area");
  add(opt.min_surface_area != d.min_surface_area, "min_surface_area");
  add(opt.max_surfaces != d.max_surfaces, "max_surfaces");
  return out;
}

void colour_vertices(
  navmap_ros_interfaces::msg::NavMap & msg, const PointSet & pts, float res, float tol)
{
  const std::size_t nv = msg.positions_x.size();
  if (!pts.coloured() || nv == 0 || pts.size() == 0) {return;}
  struct P {float x, y, z; std::uint8_t r, g, b;};
  std::unordered_map<std::uint64_t, std::vector<P>> cells;
  cells.reserve(pts.size() / 2 + 1);
  for (std::size_t k = 0; k < pts.size(); ++k) {
    const auto i = static_cast<std::int64_t>(std::floor(pts.xyz[3 * k] / res));
    const auto j = static_cast<std::int64_t>(std::floor(pts.xyz[3 * k + 1] / res));
    cells[cell_key(i, j)].push_back(
      {pts.xyz[3 * k], pts.xyz[3 * k + 1], pts.xyz[3 * k + 2], pts.rgb[3 * k], pts.rgb[3 * k + 1],
        pts.rgb[3 * k + 2]});
  }
  msg.has_vertex_rgba = true;
  msg.colors_r.assign(nv, 128);
  msg.colors_g.assign(nv, 128);
  msg.colors_b.assign(nv, 128);
  msg.colors_a.assign(nv, 255);
  for (std::size_t v = 0; v < nv; ++v) {
    const float vx = msg.positions_x[v], vy = msg.positions_y[v], vz = msg.positions_z[v];
    const auto ci = static_cast<std::int64_t>(std::floor(vx / res));
    const auto cj = static_cast<std::int64_t>(std::floor(vy / res));
    const P * best = nullptr;
    float best_d = 1e30f;
    // the vertex sits on the mesher's lattice, which may fall on the border of our cells: look at
    // the 3 x 3 around it, then at the 5 x 5 if no point of its height is there
    for (int reach = 1; reach <= 2 && !best; ++reach) {
      for (int di = -reach; di <= reach; ++di) {
        for (int dj = -reach; dj <= reach; ++dj) {
          const auto it = cells.find(cell_key(ci + di, cj + dj));
          if (it == cells.end()) {continue;}
          for (const auto & p : it->second) {
            const float dz = std::abs(p.z - vz);
            if (dz > tol) {continue;}
            const float d = (p.x - vx) * (p.x - vx) + (p.y - vy) * (p.y - vy) + dz * dz;
            if (d < best_d) {best_d = d; best = &p;}
          }
        }
      }
    }
    if (best) {
      msg.colors_r[v] = best->r;
      msg.colors_g[v] = best->g;
      msg.colors_b[v] = best->b;
    }
  }
}

Report cloud_to_navmap(
  const std::vector<std::string> & inputs, const Options & opt,
  navmap_ros_interfaces::msg::NavMap & out)
{
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();
  auto say = [&](const std::string & s) {if (opt.log) {opt.log(s);}};
  if (inputs.empty()) {throw std::runtime_error("no input point cloud");}
  if (!(opt.resolution > 0.0f)) {throw std::runtime_error("resolution must be positive");}

  Report rep;
  std::vector<PcdInfo> infos;
  for (const auto & f : inputs) {
    infos.push_back(pcd_info(f));
    rep.points_in_files += infos.back().points;
  }
  say(sformat("%.1f million points in %.0f file(s)", rep.points_in_files / 1e6, inputs.size()));

  // An organised cloud (a grid, as navmap_tools' python side writes) has its connectivity: its
  // own mesher, no reduction. That mesher takes only the resolution and the slope: with any other
  // option the grid is meshed as points, so that the option is applied
  const auto grid_ignored = options_the_grid_mesher_ignores(opt);
  if (infos.size() == 1 && infos[0].height > 1 && !grid_ignored.empty()) {
    say("organised cloud meshed as points, for " + grid_ignored);
  }
  if (infos.size() == 1 && infos[0].height > 1 && grid_ignored.empty()) {
    rep.organised = true;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(inputs[0], cloud) != 0) {
      throw std::runtime_error("PCL cannot load " + inputs[0]);
    }
    navmap_ros::BuildParams bp;
    bp.resolution = opt.resolution;
    bp.max_slope_deg = opt.max_slope_deg;
    navmap_ros::from_regular_grid(cloud, out, bp);
    rep.points_used = rep.points_meshed = cloud.size();
    rep.warnings.push_back("organised cloud: meshed as a regular grid, no colour");
  } else {
    float voxel = opt.voxel;
    if (voxel < 0.0f) {
      voxel = rep.points_in_files <= opt.raw_below ? 0.0f : 0.25f * opt.resolution;
    }
    rep.voxelized = voxel > 0.0f;
    rep.voxel = voxel;
    const float vz = std::min(voxel, 0.5f * opt.max_dz);
    VoxelCloud vox(std::max(voxel, 1e-3f), std::max(vz, 1e-3f));
    PointSet raw;
    std::size_t seen = 0, next_report = 20000000;
    for (std::size_t n = 0; n < inputs.size(); ++n) {
      say("reading " + inputs[n]);
      read_pcd(inputs[n], [&](const PcdChunk & c) {
          const bool col = !c.rgb.empty();
          for (std::size_t k = 0; k < c.size(); ++k) {
            const float x = c.xyz[3 * k], y = c.xyz[3 * k + 1], z = c.xyz[3 * k + 2];
            if (z < opt.zmin || z > opt.zmax) {continue;}
            if (opt.crop && (x < opt.xmin || x > opt.xmax || y < opt.ymin || y > opt.ymax)) {
              continue;
            }
            ++rep.points_used;
            const std::uint8_t * rgb = col ? &c.rgb[3 * k] : nullptr;
            if (rep.voxelized) {
              vox.add(x, y, z, rgb);
            } else {
              // rgb stays aligned with xyz: once a point has colour, the points without one
              // (before and after it, from other files) get kNoColour
              const std::size_t before = raw.count.size();
              raw.xyz.insert(raw.xyz.end(), {x, y, z});
              raw.count.push_back(1);
              if (col && raw.rgb.empty() && before > 0) {raw.rgb.assign(3 * before, kNoColour);}
              if (col) {
                raw.rgb.insert(raw.rgb.end(), {rgb[0], rgb[1], rgb[2]});
              } else if (!raw.rgb.empty()) {
                raw.rgb.insert(raw.rgb.end(), {kNoColour, kNoColour, kNoColour});
              }
            }
          }
          seen += c.size();
          if (seen >= next_report) {
            say(sformat("  %.0f million points read, %.0f thousand voxels", seen / 1e6,
              vox.voxels() / 1e3));
            next_report += 20000000;
          }
        });
    }
    if (rep.points_used < 3) {throw std::runtime_error("fewer than 3 points inside the window");}
    PointSet pts = rep.voxelized ? vox.centroids(opt.min_voxel_points) : std::move(raw);
    rep.voxels = rep.voxelized ? vox.voxels() : 0;
    rep.dropped = vox.dropped();
    if (rep.dropped > 0) {
      rep.warnings.push_back(sformat(
          "%.0f points were more than %.0f km from the first point read and were left out",
          static_cast<double>(rep.dropped), std::ldexp(static_cast<double>(vox.voxel_xy()), 20) /
          1e3));
    }
    rep.points_meshed = pts.size();
    if (rep.voxelized) {
      say(sformat("voxels of %.3f m: %.1f million points -> %.2f million", voxel,
          rep.points_used / 1e6,
        pts.size() / 1e6));
    }
    if (pts.size() < 3) {throw std::runtime_error("fewer than 3 points left after the reduction");}
    if (opt.recenter) {
      float x0 = 1e30f, x1 = -1e30f, y0 = 1e30f, y1 = -1e30f;
      for (std::size_t k = 0; k < pts.size(); ++k) {
        x0 = std::min(x0, pts.xyz[3 * k]); x1 = std::max(x1, pts.xyz[3 * k]);
        y0 = std::min(y0, pts.xyz[3 * k + 1]); y1 = std::max(y1, pts.xyz[3 * k + 1]);
      }
      // a whole number of cells, so that the lattice of the map does not move
      rep.offset_x = -std::round(0.5 * (x0 + x1) / opt.resolution) * opt.resolution;
      rep.offset_y = -std::round(0.5 * (y0 + y1) / opt.resolution) * opt.resolution;
      for (std::size_t k = 0; k < pts.size(); ++k) {
        pts.xyz[3 * k] += static_cast<float>(rep.offset_x);
        pts.xyz[3 * k + 1] += static_cast<float>(rep.offset_y);
      }
      say(sformat("recentred: shifted by (%.1f, %.1f) m", rep.offset_x, rep.offset_y));
    }

    // How many points each cell of the map gets, to say whether the data holds this resolution
    {
      std::unordered_map<std::uint64_t, CellCount> per_cell;
      per_cell.reserve(pts.size() / 2 + 1);
      for (std::size_t k = 0; k < pts.size(); ++k) {
        const auto i = static_cast<std::int64_t>(std::floor(pts.xyz[3 * k] / opt.resolution));
        const auto j = static_cast<std::int64_t>(std::floor(pts.xyz[3 * k + 1] / opt.resolution));
        CellCount & c = per_cell[cell_key(i, j)];
        c.raw += pts.count[k];
        ++c.kept;
      }
      std::vector<std::uint32_t> raw_n, kept_n;
      raw_n.reserve(per_cell.size());
      kept_n.reserve(per_cell.size());
      for (const auto & kv : per_cell) {
        raw_n.push_back(kv.second.raw);
        kept_n.push_back(kv.second.kept);
      }
      std::nth_element(raw_n.begin(), raw_n.begin() + raw_n.size() / 2, raw_n.end());
      std::nth_element(kept_n.begin(), kept_n.begin() + kept_n.size() / 2, kept_n.end());
      rep.median_points_per_cell = raw_n[raw_n.size() / 2];
      const double density = rep.median_points_per_cell /
        (static_cast<double>(opt.resolution) * opt.resolution);
      rep.supported_resolution = density > 0 ? std::sqrt(3.0 / density) : 0.0;
      if (rep.median_points_per_cell < 3.0) {
        rep.warnings.push_back(sformat(
            "only %.1f points per cell at %.2f m: the mesher fits a surface from about 3, below "
            "that a "
            "height is a single point's. Fine if the cloud is already a grid of this resolution; "
            "otherwise holes and rough ground follow. The data supports %.2f m",
            rep.median_points_per_cell, opt.resolution, rep.supported_resolution));
      } else if (rep.voxelized && kept_n[kept_n.size() / 2] < 4) {
        rep.warnings.push_back(sformat(
            "the voxel reduction leaves %.0f points per cell: the surface fit near borders and "
            "holes suffers. Use a smaller --voxel",
            static_cast<double>(kept_n[kept_n.size() / 2])));
      }
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(pts.size());
    for (std::size_t k = 0; k < pts.size(); ++k) {
      cloud.push_back(pcl::PointXYZ(pts.xyz[3 * k], pts.xyz[3 * k + 1], pts.xyz[3 * k + 2]));
    }
    navmap_ros::BuildParams bp;
    bp.resolution = opt.resolution;
    bp.max_slope_deg = opt.max_slope_deg;
    bp.max_dz = opt.max_dz;
    bp.min_clearance = opt.min_clearance;
    bp.max_hole_area = opt.max_hole_area;
    bp.min_surface_area = opt.min_surface_area;
    bp.max_surfaces = opt.max_surfaces;
    say(sformat("meshing %.2f million points at %.2f m", pts.size() / 1e6, opt.resolution));
    navmap_ros::from_points(cloud, out, bp);
    if (opt.fill_holes > 0.0f && !out.positions_x.empty()) {
      // the holes of the floors: floor points are added there (four per cell, at the height in
      // between) and the cloud is meshed again, so that the new triangles join the old ones
      const auto nm = navmap_ros::from_msg(out);
      const auto cells = floor_hole_cells(nm, opt.resolution, opt.fill_holes);
      rep.filled_cells = cells.size();
      if (!cells.empty()) {
        const float q = 0.25f * opt.resolution;
        for (const auto & c : cells) {
          for (const auto & d : {std::pair{-q, -q}, std::pair{q, -q}, std::pair{q, q},
              std::pair{-q, q}})
          {
            cloud.push_back(pcl::PointXYZ(c.x + d.first, c.y + d.second, c.z));
            pts.xyz.insert(pts.xyz.end(), {c.x + d.first, c.y + d.second, c.z});
            pts.count.push_back(1);
            if (pts.coloured()) {pts.rgb.insert(pts.rgb.end(), {kNoColour, kNoColour, kNoColour});}
          }
        }
        say(sformat("%.0f cells of floor over holes of the map: meshing again",
            static_cast<double>(cells.size())));
        out = navmap_ros_interfaces::msg::NavMap();
        navmap_ros::from_points(cloud, out, bp);
      }
    }
    if (pts.coloured() && !out.positions_x.empty()) {
      say("colouring the vertices");
      colour_vertices(out, pts, opt.resolution, std::max(0.4f, 2.0f * opt.max_dz));
      rep.coloured = true;
    }
  }
  rep.vertices = out.positions_x.size();
  rep.triangles = out.navcels_v0.size();
  if (rep.vertices > 0) {
    float lo = out.positions_z[0], hi = lo;
    for (const float z : out.positions_z) {
      lo = std::min(lo, z); hi = std::max(hi, z);
    }
    const int bands = static_cast<int>((hi - lo) / 0.5f) + 1;
    std::vector<std::size_t> count(bands, 0);
    for (const float z : out.positions_z) {
      ++count[static_cast<int>((z - lo) / 0.5f)];
    }
    for (int b = 0; b < bands; ++b) {   // bands holding >= 2 % of the vertices
      if (count[b] * 50 >= rep.vertices) {rep.levels.emplace_back(lo + 0.5f * b, count[b]);}
    }
  }
  if (rep.vertices == 0 || rep.triangles == 0) {
    throw std::runtime_error(
            "the cloud produced an empty NavMap: check the resolution, max_slope_deg and the "
            "window against the data");
  }
  out.header.frame_id = opt.frame_id;
  rep.seconds = std::chrono::duration<double>(clock::now() - t0).count();
  return rep;
}

}  // namespace navmap_tools
