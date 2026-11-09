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

#include "navmap_tools/floor_holes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace navmap_tools
{

namespace
{
constexpr int kBias = 1 << 20;   // 21 bits per index, +-1 M cells

std::uint64_t pack2(int a, int b)
{
  return (static_cast<std::uint64_t>(a + kBias) << 21) | static_cast<std::uint64_t>(b + kBias);
}

std::pair<int, int> split(std::uint64_t k)
{
  const std::uint64_t m = (1u << 21) - 1;
  return {static_cast<int>((k >> 21) & m) - kBias, static_cast<int>(k & m) - kBias};
}

// the surfaces of the NavMap per column of the grid, sorted by height
using Columns = std::unordered_map<std::uint64_t, std::vector<float>>;

Columns raster_floors(const ::navmap::NavMap & map, float size)
{
  Columns cols;
  const auto & P = map.positions;
  for (const auto & c : map.navcels) {
    const double x0 = P.x[c.v[0]], y0 = P.y[c.v[0]], z0 = P.z[c.v[0]];
    const double x1 = P.x[c.v[1]], y1 = P.y[c.v[1]], z1 = P.z[c.v[1]];
    const double x2 = P.x[c.v[2]], y2 = P.y[c.v[2]], z2 = P.z[c.v[2]];
    const double det = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2);
    if (std::abs(det) < 1e-12) {continue;}
    const int i0 = static_cast<int>(std::floor(std::min({x0, x1, x2}) / size));
    const int i1 = static_cast<int>(std::floor(std::max({x0, x1, x2}) / size));
    const int j0 = static_cast<int>(std::floor(std::min({y0, y1, y2}) / size));
    const int j1 = static_cast<int>(std::floor(std::max({y0, y1, y2}) / size));
    if (std::min(i0, j0) < -kBias || std::max(i1, j1) >= kBias) {   // pack2's 21 bits
      throw std::runtime_error(
              "the map is too far from (0, 0) to close its holes (more than " +
              std::to_string(static_cast<std::int64_t>(std::ldexp(size, 20))) +
              " m at cells of this size): recentre it (--recenter)");
    }
    for (int j = j0; j <= j1; ++j) {
      for (int i = i0; i <= i1; ++i) {
        const double x = (i + 0.5) * size, y = (j + 0.5) * size;
        const double l0 = ((y1 - y2) * (x - x2) + (x2 - x1) * (y - y2)) / det;
        const double l1 = ((y2 - y0) * (x - x2) + (x0 - x2) * (y - y2)) / det;
        const double l2 = 1.0 - l0 - l1;
        constexpr double kEps = -1e-9;
        if (l0 < kEps || l1 < kEps || l2 < kEps) {continue;}
        cols[pack2(i, j)].push_back(static_cast<float>(l0 * z0 + l1 * z1 + l2 * z2));
      }
    }
  }
  for (auto & kv : cols) {
    std::sort(kv.second.begin(), kv.second.end());
  }
  return cols;
}
}  // namespace

std::vector<FloorCell> floor_hole_cells(const ::navmap::NavMap & map, float cell, float max_width)
{
  std::vector<FloorCell> out;
  const int span = static_cast<int>(std::round(max_width / cell));
  if (span <= 0) {return out;}
  const Columns cols = raster_floors(map, cell);

  const auto near_height = [&](std::uint64_t col, float h, float tol, float * found) {
      const auto it = cols.find(col);
      if (it == cols.end()) {return false;}
      for (const float c : it->second) {
        if (std::abs(c - h) <= tol) {
          if (found) {*found = c;}
          return true;
        }
      }
      return false;
    };

  std::unordered_map<std::uint64_t, std::vector<float>> add;
  const std::pair<int, int> dirs[2] = {{1, 0}, {0, 1}};
  for (const auto & kv : cols) {
    const auto [a0, b0] = split(kv.first);
    for (const float h : kv.second) {
      for (const auto & [da, db] : dirs) {
        // is the next cell this floor? then there is no hole this way
        if (near_height(pack2(a0 + da, b0 + db), h, 0.35f, nullptr)) {continue;}
        for (int n = 2; n <= span + 1; ++n) {   // the other end, n cells away (n - 1 empty)
          float h2 = 0.0f;
          const float tol = 0.2f + 0.6f * static_cast<float>(n) * cell;   // up to ~30 deg
          if (!near_height(pack2(a0 + n * da, b0 + n * db), h, tol, &h2)) {continue;}
          for (int i = 1; i < n; ++i) {   // fill what the stretch between lacks
            const float hi = h + (h2 - h) * static_cast<float>(i) / static_cast<float>(n);
            const auto q = pack2(a0 + i * da, b0 + i * db);
            if (near_height(q, hi, 0.25f, nullptr)) {continue;}
            auto & list = add[q];
            if (std::none_of(
                list.begin(), list.end(), [&](float c) {return std::abs(c - hi) < 0.25f;}))
            {
              list.push_back(hi);
            }
          }
          break;   // the nearest other end only
        }
      }
    }
  }
  for (const auto & [q, levels] : add) {
    const auto [a, b] = split(q);
    for (const float h : levels) {
      out.push_back(
        {(static_cast<float>(a) + 0.5f) * cell, (static_cast<float>(b) + 0.5f) * cell, h});
    }
  }
  return out;
}

}  // namespace navmap_tools
