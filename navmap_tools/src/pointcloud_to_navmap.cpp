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

// Builds a .navmap file from a point cloud (.pcd) using the existing
// navmap_ros::from_points() mesher -- the resolution/max-slope-deg
// downsampling and filtering already implemented and tested there, not
// reimplemented here. Optionally colors the resulting mesh's vertices
// (the only real "texture/RGBD" hook NavMap has, see navmap_core/NavMap.hpp
// Colors) by nearest-neighbour lookup against a parallel r,g,b CSV that
// navmap_tools' Python side writes alongside the .pcd (see pcd_writer.py for
// why that's a plain CSV and not PCL's packed-float "rgb" PCD field).

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include "navmap_ros/conversions.hpp"
#include "navmap_ros/navmap_io.hpp"

namespace
{

struct Args
{
  std::string input;
  std::string output;
  std::string colors;
  std::string frame_id = "map";
  float resolution = 1.0f;
  float max_slope_deg = 30.0f;
};

bool parse_args(int argc, char * argv[], Args & args)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char * flag) -> const char * {
        if (i + 1 >= argc) {
          std::cerr << "missing value for " << flag << "\n";
          return nullptr;
        }
        return argv[++i];
      };
    if (arg == "--input") {
      const char * v = next("--input"); if (!v) {return false;} args.input = v;
    } else if (arg == "--output") {
      const char * v = next("--output"); if (!v) {return false;} args.output = v;
    } else if (arg == "--colors") {
      const char * v = next("--colors"); if (!v) {return false;} args.colors = v;
    } else if (arg == "--frame-id") {
      const char * v = next("--frame-id"); if (!v) {return false;} args.frame_id = v;
    } else if (arg == "--resolution") {
      const char * v = next("--resolution"); if (!v) {return false;}
      args.resolution = std::stof(v);
    } else if (arg == "--max-slope-deg") {
      const char * v = next("--max-slope-deg"); if (!v) {return false;}
      args.max_slope_deg = std::stof(v);
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return false;
    }
  }
  if (args.input.empty() || args.output.empty()) {
    std::cerr << "usage: pointcloud_to_navmap --input <cloud.pcd> --output <map.navmap> "
      "[--colors <colors.csv>] [--resolution 1.0] [--max-slope-deg 30.0] "
      "[--frame-id map]\n";
    return false;
  }
  return true;
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

}  // namespace

int main(int argc, char * argv[])
{
  Args args;
  if (!parse_args(argc, argv, args)) {
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(args.input, *cloud) != 0) {
    std::cerr << "failed to load PCD: " << args.input << "\n";
    return 1;
  }
  if (cloud->empty()) {
    std::cerr << "input point cloud is empty: " << args.input << "\n";
    return 1;
  }

  navmap_ros::BuildParams params;
  params.resolution = args.resolution;
  params.max_slope_deg = args.max_slope_deg;

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

  out_msg.header.frame_id = args.frame_id;

  std::error_code ec;
  if (!navmap_ros::io::save_msg_to_file(out_msg, args.output, {}, &ec)) {
    std::cerr << "failed to save " << args.output << ": " << ec.message() << "\n";
    return 1;
  }

  std::cout << "wrote " << args.output << " (" << n_verts << " vertices, " <<
    n_tris << " triangles" << (args.colors.empty() ? "" : ", colored") << ")\n";
  return 0;
}
