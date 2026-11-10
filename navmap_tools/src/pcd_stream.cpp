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

#include "navmap_tools/pcd_stream.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "pcl/PCLPointCloud2.h"
#include "pcl/io/pcd_io.h"

namespace navmap_tools
{

namespace
{
struct Header
{
  std::vector<std::string> fields;
  std::vector<int> size, count;
  std::vector<char> type;
  std::size_t width{0}, height{1}, points{0};
  std::string data;
  std::streamoff data_start{0};
};

std::vector<std::string> words(const std::string & line)
{
  std::istringstream is(line);
  std::vector<std::string> w;
  for (std::string s; is >> s; ) {
    w.push_back(s);
  }
  return w;
}

Header parse_header(std::ifstream & f, const std::string & path)
{
  Header h;
  std::string line;
  bool have_points = false;
  while (std::getline(f, line)) {
    if (!line.empty() && line.back() == '\r') {line.pop_back();}
    const auto w = words(line);
    if (w.empty() || w[0][0] == '#') {continue;}
    if (w[0] == "FIELDS" || w[0] == "COLUMNS") {
      h.fields.assign(w.begin() + 1, w.end());
    } else if (w[0] == "SIZE") {
      for (std::size_t i = 1; i < w.size(); ++i) {
        h.size.push_back(std::stoi(w[i]));
      }
    } else if (w[0] == "TYPE") {
      for (std::size_t i = 1; i < w.size(); ++i) {
        h.type.push_back(w[i][0]);
      }
    } else if (w[0] == "COUNT") {
      for (std::size_t i = 1; i < w.size(); ++i) {
        h.count.push_back(std::stoi(w[i]));
      }
    } else if (w[0] == "WIDTH" && w.size() > 1) {
      h.width = std::stoull(w[1]);
    } else if (w[0] == "HEIGHT" && w.size() > 1) {
      h.height = std::stoull(w[1]);
    } else if (w[0] == "POINTS" && w.size() > 1) {
      h.points = std::stoull(w[1]);
      have_points = true;
    } else if (w[0] == "DATA" && w.size() > 1) {
      h.data = w[1];
      h.data_start = f.tellg();
      break;
    }
  }
  if (h.data.empty()) {throw std::runtime_error(path + ": not a PCD file (no DATA line)");}
  if (!have_points) {h.points = h.width * h.height;}
  if (h.count.empty()) {h.count.assign(h.fields.size(), 1);}
  if (h.fields.size() != h.size.size() || h.fields.size() != h.type.size() ||
    h.fields.size() != h.count.size())
  {
    throw std::runtime_error(path + ": inconsistent FIELDS/SIZE/TYPE/COUNT");
  }
  return h;
}

int index_of(const Header & h, const char * name)
{
  for (std::size_t i = 0; i < h.fields.size(); ++i) {
    if (h.fields[i] == name) {return static_cast<int>(i);}
  }
  return -1;
}

/// Where x, y, z and the colour are inside one binary record
struct Layout
{
  std::size_t record{0};
  int ox{-1}, oy{-1}, oz{-1};
  bool dbl{false};
  int orgb{-1};
  int or_{-1}, og{-1}, ob{-1};
  bool has_color() const {return orgb >= 0 || (or_ >= 0 && og >= 0 && ob >= 0);}
};

Layout layout_of(const Header & h, const std::string & path)
{
  Layout l;
  std::vector<std::size_t> off(h.fields.size());
  std::size_t at = 0;
  for (std::size_t i = 0; i < h.fields.size(); ++i) {
    off[i] = at;
    at += static_cast<std::size_t>(h.size[i]) * h.count[i];
  }
  l.record = at;
  const int ix = index_of(h, "x"), iy = index_of(h, "y"), iz = index_of(h, "z");
  if (ix < 0 || iy < 0 || iz < 0) {throw std::runtime_error(path + ": no x, y, z fields");}
  for (const int i : {ix, iy, iz}) {
    if (h.type[i] != 'F' || (h.size[i] != 4 && h.size[i] != 8)) {
      throw std::runtime_error(path + ": x, y, z must be float32 or float64");
    }
  }
  l.dbl = h.size[ix] == 8;
  l.ox = static_cast<int>(off[ix]);
  l.oy = static_cast<int>(off[iy]);
  l.oz = static_cast<int>(off[iz]);
  int c = index_of(h, "rgb");
  if (c < 0) {c = index_of(h, "rgba");}
  if (c >= 0 && h.size[c] == 4) {l.orgb = static_cast<int>(off[c]);}
  const int ir = index_of(h, "r"), ig = index_of(h, "g"), ib = index_of(h, "b");
  if (l.orgb < 0 && ir >= 0 && ig >= 0 && ib >= 0 && h.size[ir] == 1) {
    l.or_ = static_cast<int>(off[ir]);
    l.og = static_cast<int>(off[ig]);
    l.ob = static_cast<int>(off[ib]);
  }
  return l;
}

template<typename T>
T load(const std::uint8_t * p)
{
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

void extract(const Layout & l, const std::uint8_t * data, std::size_t n, PcdChunk & out)
{
  const bool color = l.has_color();
  for (std::size_t k = 0; k < n; ++k) {
    const std::uint8_t * r = data + k * l.record;
    float x, y, z;
    if (l.dbl) {
      x = static_cast<float>(load<double>(r + l.ox));
      y = static_cast<float>(load<double>(r + l.oy));
      z = static_cast<float>(load<double>(r + l.oz));
    } else {
      x = load<float>(r + l.ox);
      y = load<float>(r + l.oy);
      z = load<float>(r + l.oz);
    }
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {continue;}
    out.xyz.insert(out.xyz.end(), {x, y, z});
    if (color) {
      if (l.orgb >= 0) {   // packed 0x00RRGGBB, little endian: bytes B, G, R, A
        out.rgb.insert(out.rgb.end(), {r[l.orgb + 2], r[l.orgb + 1], r[l.orgb]});
      } else {
        out.rgb.insert(out.rgb.end(), {r[l.or_], r[l.og], r[l.ob]});
      }
    }
  }
}
}  // namespace

PcdInfo pcd_info(const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {throw std::runtime_error("cannot open " + path);}
  const Header h = parse_header(f, path);
  PcdInfo info;
  info.points = h.points;
  info.width = h.width;
  info.height = h.height;
  info.data = h.data;
  info.has_color = layout_of(h, path).has_color();
  info.streamed = h.data == "ascii" || h.data == "binary";
  return info;
}

PcdInfo read_pcd(
  const std::string & path, const std::function<void(const PcdChunk &)> & sink,
  std::size_t chunk_points)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {throw std::runtime_error("cannot open " + path);}
  const Header h = parse_header(f, path);
  const Layout l = layout_of(h, path);
  PcdInfo info;
  info.points = h.points;
  info.width = h.width;
  info.height = h.height;
  info.data = h.data;
  info.has_color = l.has_color();
  info.streamed = h.data == "ascii" || h.data == "binary";
  chunk_points = std::max<std::size_t>(chunk_points, 1);

  if (h.data == "binary") {
    std::vector<std::uint8_t> buf(chunk_points * l.record);
    std::size_t left = h.points;
    while (left > 0) {
      const std::size_t n = std::min(left, chunk_points);
      f.read(reinterpret_cast<char *>(buf.data()), static_cast<std::streamsize>(n * l.record));
      const std::size_t got = static_cast<std::size_t>(f.gcount()) / l.record;
      if (got == 0) {break;}
      PcdChunk c;
      c.xyz.reserve(got * 3);
      extract(l, buf.data(), got, c);
      if (c.size() > 0) {sink(c);}
      if (got < n) {break;}
      left -= n;
    }
  } else if (h.data == "ascii") {
    std::vector<int> tok(h.fields.size());      // first token of each field
    int at = 0;
    for (std::size_t i = 0; i < h.fields.size(); ++i) {
      tok[i] = at; at += h.count[i];
    }
    const int ix = index_of(h, "x"), iy = index_of(h, "y"), iz = index_of(h, "z");
    int ic = index_of(h, "rgb");
    if (ic < 0) {ic = index_of(h, "rgba");}
    const int ir = index_of(h, "r"), ig = index_of(h, "g"), ib = index_of(h, "b");
    const bool packed = ic >= 0 && l.orgb >= 0;
    const bool separate = !packed && l.or_ >= 0;
    std::vector<double> v(static_cast<std::size_t>(at));
    PcdChunk c;
    std::string line;
    std::size_t done = 0;
    while (done < h.points && std::getline(f, line)) {
      const char * p = line.c_str();
      char * end = nullptr;
      int n = 0;
      for (; n < at; ++n) {
        v[static_cast<std::size_t>(n)] = std::strtod(p, &end);
        if (end == p) {break;}
        p = end;
      }
      if (n < at) {continue;}
      ++done;
      const float x = static_cast<float>(v[tok[ix]]), y = static_cast<float>(v[tok[iy]]),
        z = static_cast<float>(v[tok[iz]]);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {continue;}
      c.xyz.insert(c.xyz.end(), {x, y, z});
      if (packed) {
        std::uint32_t u;
        const double dv = v[tok[ic]];
        if (h.type[ic] == 'F' && !(dv >= 0.0 && dv <= 4294967295.0 && dv == std::floor(dv))) {
          // the float whose bits are the colour, as written by printing the float
          const float fv = static_cast<float>(dv);
          std::memcpy(&u, &fv, 4);
        } else {   // an integer: the packed value itself (PCL's ASCII writer stores rgb so)
          u = static_cast<std::uint32_t>(dv);
        }
        c.rgb.insert(c.rgb.end(), {static_cast<std::uint8_t>(u >> 16),
            static_cast<std::uint8_t>(u >> 8), static_cast<std::uint8_t>(u)});
      } else if (separate) {
        c.rgb.insert(c.rgb.end(), {static_cast<std::uint8_t>(v[tok[ir]]),
            static_cast<std::uint8_t>(v[tok[ig]]), static_cast<std::uint8_t>(v[tok[ib]])});
      }
      if (c.size() >= chunk_points) {sink(c); c = PcdChunk();}
    }
    if (c.size() > 0) {sink(c);}
  } else {  // binary_compressed: the whole table is one LZF block
    pcl::PCLPointCloud2 blob;
    if (pcl::PCDReader().read(path, blob) < 0) {
      throw std::runtime_error("PCL cannot read " + path);
    }
    Layout lc;
    lc.record = blob.point_step;
    for (const auto & fl : blob.fields) {
      if (fl.name == "x") {
        lc.ox = static_cast<int>(fl.offset); lc.dbl = fl.datatype == pcl::PCLPointField::FLOAT64;
      }
      if (fl.name == "y") {lc.oy = static_cast<int>(fl.offset);}
      if (fl.name == "z") {lc.oz = static_cast<int>(fl.offset);}
      if ((fl.name == "rgb" || fl.name == "rgba") && lc.orgb < 0) {
        lc.orgb = static_cast<int>(fl.offset);
      }
    }
    if (lc.ox < 0 || lc.oy < 0 || lc.oz < 0) {
      throw std::runtime_error(path + ": no x, y, z fields");
    }
    info.has_color = lc.has_color();
    const std::size_t total = blob.width * blob.height;
    for (std::size_t at = 0; at < total; at += chunk_points) {
      const std::size_t n = std::min(chunk_points, total - at);
      PcdChunk c;
      extract(lc, blob.data.data() + at * lc.record, n, c);
      if (c.size() > 0) {sink(c);}
    }
  }
  return info;
}

}  // namespace navmap_tools
