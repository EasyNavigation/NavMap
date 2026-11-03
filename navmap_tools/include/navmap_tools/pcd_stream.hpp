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

#ifndef NAVMAP_TOOLS__PCD_STREAM_HPP_
#define NAVMAP_TOOLS__PCD_STREAM_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace navmap_tools
{

/// What the header of a .pcd says
struct PcdInfo
{
  std::size_t points{0};
  std::size_t width{0}, height{1};
  std::string data;          ///< ascii | binary | binary_compressed
  bool has_color{false};     ///< an rgb / rgba field, or separate r, g, b
  bool streamed{false};      ///< read in chunks (ascii, binary); compressed files are loaded whole
};

/// A block of points. `rgb` has 3 bytes per point, or is empty when the file has no colour.
struct PcdChunk
{
  std::vector<float> xyz;
  std::vector<std::uint8_t> rgb;
  std::size_t size() const {return xyz.size() / 3;}
};

/// Reads the header only.
PcdInfo pcd_info(const std::string & path);

/// Streams the points of a .pcd to `sink` in blocks of at most `chunk_points`. ascii and binary
/// files are never held whole in memory (a 100 million point file costs one block); the
/// binary_compressed format (LZF over the whole table) cannot be streamed and is loaded by PCL.
/// Points with a NaN coordinate are skipped. Throws std::runtime_error on an unreadable file or
/// one without x, y, z.
PcdInfo read_pcd(
  const std::string & path, const std::function<void(const PcdChunk &)> & sink,
  std::size_t chunk_points = 1u << 20);

}  // namespace navmap_tools

#endif  // NAVMAP_TOOLS__PCD_STREAM_HPP_
