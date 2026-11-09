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

#ifndef NAVMAP_TOOLS__CLOUD_TO_NAVMAP_HPP_
#define NAVMAP_TOOLS__CLOUD_TO_NAVMAP_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "navmap_tools/voxel_cloud.hpp"
#include "navmap_ros_interfaces/msg/nav_map.hpp"

namespace navmap_tools
{

struct Options
{
  /// The finest NavCel the map will have: the maximum resolution a consumer works from
  /// (for bigmap, lengths.cell_size.min). Finer is wasted for the grid engine, which rasterises at
  /// cell_size.min; coarser throws detail away for good.
  float resolution{1.0f};
  float max_slope_deg{30.0f};   ///< steeper triangles are not created (walls stay as gaps)
  float max_dz{0.25f};          ///< vertical step of one surface / spread that means clutter
  float min_clearance{1.0f};    ///< free height that separates two floors in a cell: the robot's
  float max_hole_area{1.0f};    ///< enclosed empty regions up to this (m^2) are interpolated
  /// A SLAM cloud has no points where it did not see the floor, and the mesher cannot invent them:
  /// a map of holes, and ground the robot could drive over is not connected. A stretch of empty
  /// cells up to this wide (m) between two bits of the same floor, in a row or a column, gets floor
  /// points at the height in between and the cloud is meshed again (0, the default: leave them)
  float fill_holes{0.0f};
  float min_surface_area{0.0f};  ///< pieces of the map smaller than this (m^2) are dropped
  int max_surfaces{0};          ///< keep only this many pieces, the largest (0: all)

  /// Voxel size (m) of the pre-reduction: < 0 automatic (none up to `raw_below` points, else
  /// resolution / 4: 16 voxels per cell, so only a cloud denser than that is reduced and each
  /// cell keeps enough points for the mesher's surface fit), 0 none.
  float voxel{-1.0f};
  std::size_t raw_below{5000000};
  std::uint32_t min_voxel_points{1};   ///< 2 or more also drops isolated noise points

  /// Only what is inside this window (x, y) and these heights is used
  bool crop{false};
  double xmin{0}, ymin{0}, xmax{0}, ymax{0};
  double zmin{-std::numeric_limits<double>::infinity()};
  double zmax{std::numeric_limits<double>::infinity()};

  /// Shift x, y so that the centre of the cloud's bounding box is (0, 0): a consumer that
  /// rasterises a map around its origin needs a grid centred on it, so a map far from it would
  /// waste memory. The shift is in the report.
  bool recenter{false};

  std::string frame_id{"map"};
  std::function<void(const std::string &)> log;   ///< progress; empty: silent
};

struct Report
{
  std::size_t points_in_files{0}, points_used{0};
  bool voxelized{false};
  float voxel{0.0f};
  std::size_t voxels{0}, points_meshed{0}, dropped{0};
  std::size_t filled_cells{0};   ///< cells of floor added over holes (fill_holes)
  /// Points per occupied cell of `resolution` (median) and the finest resolution the data
  /// supports with at least 3 points per cell
  double median_points_per_cell{0.0};
  double supported_resolution{0.0};
  std::size_t vertices{0}, triangles{0};
  bool coloured{false};
  bool organised{false};
  double offset_x{0.0}, offset_y{0.0};   ///< what --recenter added to x, y (subtract it to go back)
  double seconds{0.0};
  /// Heights (m) with surface in the map: the floors, from the lowest (vertices per 0.5 m band)
  std::vector<std::pair<float, std::size_t>> levels;
  std::vector<std::string> warnings;
};

/// The options set in `opt` that the mesher of organised clouds (from_regular_grid) does not
/// take, comma-separated ("" if none): with any of them an organised cloud is meshed as points.
std::string options_the_grid_mesher_ignores(const Options & opt);

/// The NavMap of one or several .pcd files (their points are joined) at `opt.resolution`.
/// Streams the files, reduces by voxel when they are big, meshes with navmap_ros::from_points
/// (several floors, slopes, clutter) and gives the mesh the colour of the points. Throws
/// std::runtime_error with the reason when nothing can be built.
Report cloud_to_navmap(
  const std::vector<std::string> & inputs, const Options & opt,
  navmap_ros_interfaces::msg::NavMap & out);

/// Colours the vertices of `msg` from the coloured points (nearest in 3D among the ones of the
/// vertex's cell and its neighbours, within a height tolerance; the rest take the nearest height).
void colour_vertices(
  navmap_ros_interfaces::msg::NavMap & msg, const PointSet & points, float resolution,
  float height_tolerance);

}  // namespace navmap_tools

#endif  // NAVMAP_TOOLS__CLOUD_TO_NAVMAP_HPP_
