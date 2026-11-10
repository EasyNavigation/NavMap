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

#include "navmap_tools/voxel_cloud.hpp"

#include <cmath>

namespace navmap_tools
{

namespace
{
constexpr std::int64_t kOffXY = 1 << 20, kOffZ = 1 << 21;   // 21 + 21 + 22 bits
constexpr std::int64_t kMaxXY = (1 << 21) - 1, kMaxZ = (1 << 22) - 1;
}  // namespace

VoxelCloud::VoxelCloud(float voxel_xy, float voxel_z)
: vx_(voxel_xy), vz_(voxel_z)
{
  grid_.reserve(1u << 20);
}

bool VoxelCloud::add(float x, float y, float z, const std::uint8_t * rgb)
{
  const std::int64_t ix = static_cast<std::int64_t>(std::floor(x / vx_));
  const std::int64_t iy = static_cast<std::int64_t>(std::floor(y / vx_));
  const std::int64_t iz = static_cast<std::int64_t>(std::floor(z / vz_));
  if (!anchored_) {   // a cloud far from (0, 0) (UTM) is in range around its own first point
    bx_ = ix; by_ = iy; bz_ = iz;
    anchored_ = true;
  }
  const std::int64_t kx = ix - bx_ + kOffXY, ky = iy - by_ + kOffXY, kz = iz - bz_ + kOffZ;
  if (kx < 0 || kx > kMaxXY || ky < 0 || ky > kMaxXY || kz < 0 || kz > kMaxZ) {
    ++dropped_;
    return false;
  }
  const std::uint64_t key = (static_cast<std::uint64_t>(kx) << 43) |
    (static_cast<std::uint64_t>(ky) << 22) | static_cast<std::uint64_t>(kz);
  Acc & a = grid_[key];
  a.sx += x - static_cast<float>(ix) * vx_;
  a.sy += y - static_cast<float>(iy) * vx_;
  a.sz += z - static_cast<float>(iz) * vz_;
  ++a.n;
  if (rgb) {
    a.r += rgb[0]; a.g += rgb[1]; a.b += rgb[2];
    ++a.nc;
    coloured_ = true;
  }
  return true;
}

PointSet VoxelCloud::centroids(std::uint32_t min_points) const
{
  PointSet out;
  out.xyz.reserve(grid_.size() * 3);
  out.count.reserve(grid_.size());
  if (coloured_) {out.rgb.reserve(grid_.size() * 3);}
  for (const auto & [key, a] : grid_) {
    if (a.n < min_points) {continue;}
    const std::int64_t ix = static_cast<std::int64_t>(key >> 43) - kOffXY + bx_;
    const std::int64_t iy = static_cast<std::int64_t>((key >> 22) & kMaxXY) - kOffXY + by_;
    const std::int64_t iz = static_cast<std::int64_t>(key & kMaxZ) - kOffZ + bz_;
    const float inv = 1.0f / static_cast<float>(a.n);
    out.xyz.push_back(static_cast<float>(ix) * vx_ + a.sx * inv);
    out.xyz.push_back(static_cast<float>(iy) * vx_ + a.sy * inv);
    out.xyz.push_back(static_cast<float>(iz) * vz_ + a.sz * inv);
    out.count.push_back(a.n);
    if (coloured_) {   // the mean of the points with colour (a voxel of none: kNoColour)
      const bool c = a.nc > 0;
      out.rgb.push_back(c ? static_cast<std::uint8_t>(a.r / a.nc) : kNoColour);
      out.rgb.push_back(c ? static_cast<std::uint8_t>(a.g / a.nc) : kNoColour);
      out.rgb.push_back(c ? static_cast<std::uint8_t>(a.b / a.nc) : kNoColour);
    }
  }
  return out;
}

}  // namespace navmap_tools
