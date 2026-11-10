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

// Builds a .navmap file from a point cloud (.pcd).
//
// Two paths, chosen by the input:
//  * an organized (grid-shaped) cloud, which is what navmap_tools' Python side writes, or --colors:
//    the cloud is loaded whole and meshed by navmap_ros::from_regular_grid / from_points, the
//    connectivity of the grid being already known. --colors colours the mesh's vertices (the only
//    real "texture/RGBD" hook NavMap has, see navmap_core/NavMap.hpp Colors) by nearest-neighbour
//    lookup against a parallel r,g,b CSV (see pcd_writer.py for why that is a plain CSV and not
//    PCL's packed-float "rgb" PCD field).
//  * any other cloud (a SLAM map, a LiDAR scan, tens of millions of points), or any option of the
//    second group: streamed, never held whole, reduced by voxel, and meshed in several floors by
//    navmap_tools::cloud_to_navmap, which also closes the holes a SLAM leaves in the floor.

#include <array>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "navmap_ros/conversions.hpp"
#include "navmap_ros/navmap_io.hpp"
#include "navmap_tools/cloud_to_navmap.hpp"

namespace
{

struct Args
{
  std::vector<std::string> inputs;
  std::string output;
  std::string colors;
  navmap_tools::Options opt;
  bool streaming_options{false};   // an option only the streaming path has was given
  bool georef{false};
  double origin[3] = {0, 0, 0};
};

void usage()
{
  std::cout <<
    "usage: pointcloud_to_navmap --input <cloud.pcd> [--input <more.pcd> ...] "
    "--output <map.navmap> [options]\n"
    "\n"
    "  --resolution m        finest NavCel (default 1.0); for bigmap, lengths.cell_size.min\n"
    "  --max-slope-deg d     triangles steeper than this are not made (30)\n"
    "  --frame-id id         frame of the map (map)\n"
    "  --colors file.csv     colour the vertices from a row-aligned r,g,b CSV (one input, loaded\n"
    "                        whole; not combined with the options below)\n"
    "\n"
    "Options of the streaming mesher. Used for every cloud that is not organized (a grid, as\n"
    "navmap_gis_tool writes), or when any of them is given. ascii and binary .pcd files are\n"
    "streamed (never held whole); big clouds are reduced by voxel first, so 100 million points\n"
    "need as much memory as the map's extent, not as the points. The mesh keeps several floors,\n"
    "the slopes and what is too steep, and takes the colour of the points (rgb field).\n"
    "\n"
    "  --max-dz m            vertical step of a surface / spread that means clutter (0.25)\n"
    "  --min-clearance m     free height that separates two floors in a cell, the robot's (1.0)\n"
    "  --max-hole-area m2    enclosed empty regions up to this are interpolated (1.0)\n"
    "  --fill-holes m        close the holes in the floors up to this wide, between two bits\n"
    "                        of the same floor (0, the default: leave them). A SLAM cloud has\n"
    "                        no floor points where it did not see it, and ground the robot\n"
    "                        could drive over is cut; this adds synthetic floor there\n"
    "  --min-surface-area m2 drop pieces of the map smaller than this (0)\n"
    "  --max-surfaces n      keep only the n largest pieces (0: all)\n"
    "  --voxel m             pre-reduction voxel: automatic by default (none up to 5 million\n"
    "                        points, resolution / 4 above), 0 = never\n"
    "  --min-voxel-points n  drop voxels with fewer points (noise; 1)\n"
    "  --crop x0,y0,x1,y1    only this window of the cloud\n"
    "  --zmin m --zmax m     only these heights (a ceiling above the robot, say)\n"
    "  --origin lat,lon,alt  where the map's (0, 0, 0) is: writes <map>.yaml as georeferenced\n"
    "  --recenter            shift the map so that its centre is (0, 0) (the shift is written\n"
    "                        in <map>.yaml as offset_x / offset_y): bigmap rasterises around its\n"
    "                        origin\n"
    "\n"
    "With --origin or --recenter, <map>.yaml is written next to the map (origin, resolution,\n"
    "offset), in the format bigmap reads. Without them nothing but the map is written.\n";
}

bool numbers(const std::string & s, double * v, int n)
{
  std::vector<double> out;
  std::size_t at = 0;
  while (at <= s.size()) {
    const auto c = s.find(',', at);
    out.push_back(std::atof(s.substr(at, c == std::string::npos ? c : c - at).c_str()));
    if (c == std::string::npos) {break;}
    at = c + 1;
  }
  if (static_cast<int>(out.size()) != n) {return false;}
  for (int i = 0; i < n; ++i) {
    v[i] = out[static_cast<std::size_t>(i)];
  }
  return true;
}

// 0: go on, 1: done (--help), 2: bad arguments
int parse_args(int argc, char * argv[], Args & args)
{
  auto & opt = args.opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
        if (i + 1 >= argc) {std::cerr << "missing value for " << a << "\n"; std::exit(2);}
        return argv[++i];
      };
    if (a == "--help" || a == "-h") {usage(); return 1;} else if (a == "--input") {
      args.inputs.push_back(next());
    } else if (a == "--output") {args.output = next();} else if (a == "--colors") {
      args.colors = next();
    } else if (a == "--frame-id") {opt.frame_id = next();} else if (a == "--resolution") {
      opt.resolution = std::stof(next());
    } else if (a == "--max-slope-deg") {opt.max_slope_deg = std::stof(next());} else {
      args.streaming_options = true;
      if (a == "--max-dz") {opt.max_dz = std::stof(next());} else if (a == "--min-clearance") {
        opt.min_clearance = std::stof(next());
      } else if (a == "--max-hole-area") {
        opt.max_hole_area = std::stof(next());
      } else if (a == "--fill-holes") {
        opt.fill_holes = std::stof(next());
      } else if (a == "--min-surface-area") {
        opt.min_surface_area = std::stof(next());
      } else if (a == "--max-surfaces") {
        opt.max_surfaces = std::stoi(next());
      } else if (a == "--voxel") {
        opt.voxel = std::stof(next());
      } else if (a == "--min-voxel-points") {
        opt.min_voxel_points = static_cast<std::uint32_t>(std::stoul(next()));
      } else if (a == "--zmin") {opt.zmin = std::stod(next());} else if (a == "--zmax") {
        opt.zmax = std::stod(next());
      } else if (a == "--recenter") {opt.recenter = true;} else if (a == "--crop") {
        double w[4];
        if (!numbers(next(), w, 4)) {std::cerr << "--crop x0,y0,x1,y1\n"; return 2;}
        opt.crop = true; opt.xmin = w[0]; opt.ymin = w[1]; opt.xmax = w[2]; opt.ymax = w[3];
      } else if (a == "--origin") {
        if (!numbers(next(), args.origin, 3)) {std::cerr << "--origin lat,lon,alt\n"; return 2;}
        args.georef = true;
      } else {
        std::cerr << "unknown argument: " << a << " (see --help)\n";
        return 2;
      }
    }
  }
  if (args.inputs.empty() || args.output.empty()) {
    usage();
    return 2;
  }
  if (!args.colors.empty() && (args.streaming_options || args.inputs.size() > 1)) {
    std::cerr << "--colors takes one input and none of the streaming options (see --help)\n";
    return 2;
  }
  return 0;
}

bool is_organized(const std::string & path)
{
  pcl::PCLPointCloud2 header;
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  int version = 0, type = 0;
  unsigned int data_idx = 0;
  if (pcl::PCDReader().readHeader(path, header, origin, orientation, version, type, data_idx) < 0) {
    return false;   // the streaming reader says what is wrong with the file
  }
  return header.height > 1;
}

// One "r,g,b" triple per line, same row order as the input .pcd.
std::vector<std::array<uint8_t, 3>> load_colors_csv(const std::string & path)
{
  std::vector<std::array<uint8_t, 3>> colors;
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error("cannot open colors CSV: " + path);
  }
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) {continue;}
    std::istringstream iss(line);
    std::string r_s, g_s, b_s;
    if (!std::getline(iss, r_s, ',') || !std::getline(iss, g_s, ',') ||
      !std::getline(iss, b_s, ','))
    {
      throw std::runtime_error("malformed colors CSV line: " + line);
    }
    colors.push_back(
      {static_cast<uint8_t>(std::stoi(r_s)), static_cast<uint8_t>(std::stoi(g_s)),
        static_cast<uint8_t>(std::stoi(b_s))});
  }
  return colors;
}

int run_whole_cloud(const Args & args)
{
  const std::string & input = args.inputs.front();
  const auto & opt = args.opt;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(input, *cloud) != 0) {
    std::cerr << "failed to load PCD: " << input << "\n";
    return 1;
  }
  if (cloud->empty()) {
    std::cerr << "input point cloud is empty: " << input << "\n";
    return 1;
  }

  navmap_ros::BuildParams params;
  params.resolution = opt.resolution;
  params.max_slope_deg = opt.max_slope_deg;

  navmap_ros_interfaces::msg::NavMap out_msg;
  // navmap_tools always writes its own .pcd as an *organized* (grid-shaped)
  // cloud (see pcd_writer.write_pcd_xyz), so the connectivity between
  // adjacent samples is already fully known -- from_regular_grid meshes it
  // deterministically from the grid indices with no neighbor search, which
  // both avoids the search-radius-driven gaps from_points can leave on an
  // evenly-sampled, fully navigable grid and produces far fewer redundant
  // triangles. Older/foreign .pcd files (unorganized, height == 1, e.g. a
  // hand-collected point cloud) still go through the generic from_points
  // mesher unchanged.
  navmap::NavMap nm = cloud->height > 1 ?
    navmap_ros::from_regular_grid(*cloud, out_msg, params) :
    navmap_ros::from_points(*cloud, out_msg, params);
  (void)nm;

  const size_t n_verts = out_msg.positions_x.size();
  const size_t n_tris = out_msg.navcels_v0.size();
  if (n_verts == 0 || n_tris == 0) {
    std::cerr << "meshing produced an empty NavMap (" << n_verts << " vertices, " <<
      n_tris << " triangles) -- check --resolution/--max-slope-deg against the input\n";
    return 1;
  }

  if (!args.colors.empty()) {
    std::vector<std::array<uint8_t, 3>> colors;
    try {
      colors = load_colors_csv(args.colors);
    } catch (const std::exception & e) {
      std::cerr << e.what() << "\n";
      return 1;
    }
    if (colors.size() != cloud->size()) {
      std::cerr << "colors CSV has " << colors.size() << " rows but the input cloud has " <<
        cloud->size() << " points; they must be row-aligned\n";
      return 1;
    }

    pcl::search::KdTree<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    out_msg.has_vertex_rgba = true;
    out_msg.colors_r.resize(n_verts);
    out_msg.colors_g.resize(n_verts);
    out_msg.colors_b.resize(n_verts);
    out_msg.colors_a.resize(n_verts, 255);

    std::vector<int> idx(1);
    std::vector<float> dist(1);
    for (size_t i = 0; i < n_verts; ++i) {
      pcl::PointXYZ query(
        out_msg.positions_x[i], out_msg.positions_y[i], out_msg.positions_z[i]);
      if (kdtree.nearestKSearch(query, 1, idx, dist) > 0) {
        const auto & c = colors[static_cast<size_t>(idx[0])];
        out_msg.colors_r[i] = c[0];
        out_msg.colors_g[i] = c[1];
        out_msg.colors_b[i] = c[2];
      }
    }
  }

  out_msg.header.frame_id = opt.frame_id;

  std::error_code ec;
  if (!navmap_ros::io::save_msg_to_file(out_msg, args.output, {}, &ec)) {
    std::cerr << "failed to save " << args.output << ": " << ec.message() << "\n";
    return 1;
  }

  std::cout << "wrote " << args.output << " (" << n_verts << " vertices, " <<
    n_tris << " triangles" << (args.colors.empty() ? "" : ", colored") << ")\n";
  return 0;
}

int run_streaming(Args & args)
{
  auto & opt = args.opt;
  opt.log = [](const std::string & s) {std::cerr << "pointcloud_to_navmap: " << s << "\n";};

  navmap_ros_interfaces::msg::NavMap msg;
  navmap_tools::Report rep;
  try {
    rep = navmap_tools::cloud_to_navmap(args.inputs, opt, msg);
  } catch (const std::exception & e) {
    std::cerr << "pointcloud_to_navmap: " << e.what() << "\n";
    return 1;
  }
  std::error_code ec;
  if (!navmap_ros::io::save_msg_to_file(msg, args.output, {}, &ec)) {
    std::cerr << "pointcloud_to_navmap: cannot save " << args.output << ": " << ec.message() <<
      "\n";
    return 1;
  }
  // <map>.yaml next to it, in the format bigmap reads: where the map's (0, 0, 0) is. Only when
  // there is something to say (an origin, a shift), so that no other <map>.yaml is overwritten
  if (args.georef || opt.recenter) {
    std::string yaml = args.output;
    const auto dot = yaml.rfind('.');
    yaml = (dot == std::string::npos ? yaml : yaml.substr(0, dot)) + ".yaml";
    std::ofstream f(yaml, std::ios::trunc);
    f.precision(10);
    f << "# Position of the map's (0, 0, 0). Without this file a map is loaded as a plain NavMap\n"
      << "# without geolocation.\n"
      << "georeferenced: " << (args.georef ? "true" : "false") << "\n"
      << "origin_lat: " << args.origin[0] << "\n"
      << "origin_lon: " << args.origin[1] << "\n"
      << "reference_altitude: " << args.origin[2] << "\n"
      << "min_resolution: " << opt.resolution << "\n"
      << "# --recenter: x, y of the cloud + offset = x, y of this map\n"
      << "offset_x: " << rep.offset_x << "\n"
      << "offset_y: " << rep.offset_y << "\n";
    std::cerr << "pointcloud_to_navmap: wrote " << yaml << "\n";
  }
  std::printf(
    "wrote %s: %zu vertices, %zu triangles, %.2f m%s%s in %.1f s\n", args.output.c_str(),
    rep.vertices, rep.triangles, static_cast<double>(opt.resolution),
    rep.coloured ? ", coloured" : "", rep.voxelized ? ", voxel-reduced" : "", rep.seconds);
  std::printf(
    "  %zu points in the files, %zu used, %zu meshed; median %.1f points per cell"
    " (this data holds %.2f m)\n", rep.points_in_files, rep.points_used, rep.points_meshed,
    rep.median_points_per_cell, rep.supported_resolution);
  std::printf("  surface at (m, vertices per 0.5 m band, >= 2 %%):");
  for (const auto & [z, n] : rep.levels) {
    std::printf(" %.1f:%zu", static_cast<double>(z), n);
  }
  std::printf("\n");
  for (const auto & w : rep.warnings) {
    std::printf("  WARNING: %s\n", w.c_str());
  }
  return 0;
}

}  // namespace

int main(int argc, char * argv[])
{
  Args args;
  if (const int rc = parse_args(argc, argv, args)) {
    return rc == 1 ? 0 : 2;
  }
  const bool whole = !args.colors.empty() ||
    (args.inputs.size() == 1 && !args.streaming_options && is_organized(args.inputs.front()));
  return whole ? run_whole_cloud(args) : run_streaming(args);
}
