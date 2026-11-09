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

#ifndef NAVMAP_TOOLS__VOXEL_CLOUD_HPP_
#define NAVMAP_TOOLS__VOXEL_CLOUD_HPP_

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace navmap_tools
{

/// Points with the colour (3 bytes each, or empty) and how many raw points each one stands for.
/// Colour given to a point without one (and to floor added over holes) when others have colour
constexpr std::uint8_t kNoColour = 128;

struct PointSet
{
  std::vector<float> xyz;
  std::vector<std::uint8_t> rgb;
  std::vector<std::uint32_t> count;
  std::size_t size() const {return xyz.size() / 3;}
  bool coloured() const {return !rgb.empty();}
};

/// Streaming voxel grid: a hash of occupied voxels, each holding the sum of its points. Memory
/// grows with the number of occupied voxels (the extent of the map over the voxel size), not with
/// the number of points: a hundred million points over a 1 km^2 floor are a few million voxels.
/// A voxel is replaced by the centroid of its points, which keeps planes planes and the vertical
/// spread of clutter (what the mesher fits and tests), unlike keeping an arbitrary point.
class VoxelCloud
{
public:
  VoxelCloud(float voxel_xy, float voxel_z);

  /// `rgb`: 3 bytes, or nullptr for a point without colour. False (the point is dropped) if its
  /// voxel is more than 2^20 voxels away from the first point's in x or y (2^21 in z).
  bool add(float x, float y, float z, const std::uint8_t * rgb);

  std::size_t voxels() const {return grid_.size();}
  std::size_t dropped() const {return dropped_;}
  float voxel_xy() const {return vx_;}

  /// Centroids of the voxels with at least `min_points` points
  PointSet centroids(std::uint32_t min_points = 1) const;

private:
  struct Acc
  {
    float sx{0}, sy{0}, sz{0};            // offsets from the voxel corner, summed
    std::uint32_t n{0}, nc{0}, r{0}, g{0}, b{0};   // nc: points with colour
  };
  float vx_, vz_;
  bool anchored_{false};
  std::int64_t bx_{0}, by_{0}, bz_{0};   // voxel of the first point: keys are relative to it
  bool coloured_{false};
  std::size_t dropped_{0};
  std::unordered_map<std::uint64_t, Acc> grid_;
};

}  // namespace navmap_tools

#endif  // NAVMAP_TOOLS__VOXEL_CLOUD_HPP_
