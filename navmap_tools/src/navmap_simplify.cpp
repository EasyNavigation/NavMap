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

// Simplifies a .navmap keeping its layers (navmap_core/Simplify.hpp):
//   navmap_simplify in.navmap out.navmap [--max-error m] [--max-edge m] [--min-quality q]
//                   [--max-normal-change deg] [--layer name]... [--ignore-layer name]...

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "navmap_core/Simplify.hpp"
#include "navmap_ros/navmap_io.hpp"

namespace
{

int usage()
{
  std::fprintf(
    stderr,
    "usage: navmap_simplify in.navmap out.navmap [--max-error m] [--max-edge m]\n"
    "         [--min-quality q] [--max-normal-change deg] [--layer name]...\n"
    "         [--ignore-layer name]...\n"
    "  Merges NavCels where the surface is flat (within --max-error) and every layer (or the\n"
    "  --layer ones, minus the --ignore-layer ones) has the same value. Borders between\n"
    "  values and open edges are kept. Ignored layers keep the value of the surviving NavCel,\n"
    "  except \"elevation\", recomputed as the mean height of each NavCel.\n");
  return 2;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {return usage();}
  const std::string in = argv[1], out = argv[2];
  navmap::SimplifyParams p;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    if (i + 1 >= argc) {return usage();}
    char * end = nullptr;
    const char * v = argv[++i];
    const float f = std::strtof(v, &end);
    const bool num = end != v && *end == '\0';
    if (a == "--max-error" && num && f >= 0.0f) {
      p.max_error = f;
    } else if (a == "--max-edge" && num && f > 0.0f) {
      p.max_edge_length = f;
    } else if (a == "--min-quality" && num && f >= 0.0f && f <= 1.0f) {
      p.min_quality = f;
    } else if (a == "--max-normal-change" && num && f >= 0.0f && f <= 180.0f) {
      p.max_normal_change = f;
    } else if (a == "--layer") {
      p.layers.emplace_back(v);
    } else if (a == "--ignore-layer") {
      p.ignore_layers.emplace_back(v);
    } else {
      return usage();
    }
  }
  navmap::NavMap nm;
  std::error_code ec;
  if (!navmap_ros::io::load_from_file(in, nm, &ec)) {
    std::fprintf(stderr, "cannot load %s: %s\n", in.c_str(), ec.message().c_str());
    return 1;
  }
  const auto t0 = std::chrono::steady_clock::now();
  const auto st = navmap::simplify(nm, p);
  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();
  // from_points' "elevation" is the mean height of the NavCel: recompute it if it was ignored
  if (nm.has_layer("elevation") && nm.layer_type_name("elevation") == "float" &&
    std::find(p.ignore_layers.begin(), p.ignore_layers.end(), "elevation") != p.ignore_layers.end())
  {
    for (navmap::NavCelId c = 0; c < nm.navcels.size(); ++c) {
      nm.layer_set<float>("elevation", c, nm.navcel_centroid(c).z());
    }
  }
  if (!navmap_ros::io::save_to_file(nm, out, {}, &ec)) {
    std::fprintf(stderr, "cannot save %s: %s\n", out.c_str(), ec.message().c_str());
    return 1;
  }
  std::printf(
    "NavCels %zu -> %zu (%.1f %%), vertices %zu -> %zu, %.0f ms; file %s: %ju bytes\n",
    st.navcels_before, st.navcels_after,
    st.navcels_before ? 100.0 * st.navcels_after / st.navcels_before : 0.0,
    st.vertices_before, st.vertices_after, ms, out.c_str(),
    static_cast<std::uintmax_t>(std::filesystem::file_size(out)));
  return 0;
}
