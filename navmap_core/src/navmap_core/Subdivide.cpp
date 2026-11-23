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

#include "navmap_core/Subdivide.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace navmap
{

namespace
{

constexpr uint32_t kNone = std::numeric_limits<uint32_t>::max();
using Tri = std::array<uint32_t, 3>;
using Rgba = std::array<uint8_t, 4>;

// Calls f(view) with the typed view of @p base (no-op for an unknown type).
template<class F>
void with_view(const std::shared_ptr<LayerViewBase> & base, F && f)
{
  if (auto v = std::dynamic_pointer_cast<LayerView<uint8_t>>(base)) {f(*v); return;}
  if (auto v = std::dynamic_pointer_cast<LayerView<float>>(base)) {f(*v); return;}
  if (auto v = std::dynamic_pointer_cast<LayerView<double>>(base)) {f(*v); return;}
  if (auto v = std::dynamic_pointer_cast<LayerView<int16_t>>(base)) {f(*v); return;}
}

uint64_t edge_key(uint32_t a, uint32_t b)
{
  return a < b ? (static_cast<uint64_t>(a) << 32) | b : (static_cast<uint64_t>(b) << 32) | a;
}

// The vertices of a map being subdivided: positions and (optional) colours, growing
struct Vertices
{
  std::vector<Eigen::Vector3f> p;
  std::vector<Rgba> c;
  bool colored{false};

  explicit Vertices(const NavMap & nm)
  {
    const std::size_t n = nm.positions.size();
    p.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
      p[i] = nm.positions.at(static_cast<PointId>(i));
    }
    colored = nm.colors && nm.colors->r.size() == n;
    if (colored) {
      c.resize(n);
      for (std::size_t i = 0; i < n; ++i) {
        c[i] = {nm.colors->r[i], nm.colors->g[i], nm.colors->b[i], nm.colors->a[i]};
      }
    }
  }

  // A new vertex at a + t (b - a)
  uint32_t add_between(uint32_t a, uint32_t b, float t)
  {
    p.push_back(p[a] + t * (p[b] - p[a]));
    if (colored) {
      Rgba m;
      for (int k = 0; k < 4; ++k) {
        m[k] = static_cast<uint8_t>(std::lround(c[a][k] + t * (c[b][k] - c[a][k])));
      }
      c.push_back(m);
    }
    return static_cast<uint32_t>(p.size() - 1);
  }
};

// Replaces the NavCels of @p nm by @p tris, each coming from the old NavCel parent[i]: its layer
// values, its surface; a link goes to the first NavCel coming from each of its ends
SubdivideStats replace(
  NavMap & nm, const std::vector<Tri> & tris, const std::vector<NavCelId> & parent,
  const Vertices & verts, SubdivideStats st)
{
  const std::size_t old_n = nm.navcels.size(), n = tris.size();
  std::vector<std::size_t> surf(old_n, NavMap::kNoSurface);
  for (std::size_t s = 0; s < nm.surfaces.size(); ++s) {
    for (const NavCelId c : nm.surfaces[s].navcels) {
      if (c < old_n) {surf[c] = s;}
    }
    nm.surfaces[s].navcels.clear();
  }
  std::vector<NavCel> cels(n);
  std::vector<NavCelId> first(old_n, kNone);
  for (std::size_t i = 0; i < n; ++i) {
    for (int k = 0; k < 3; ++k) {
      cels[i].v[k] = tris[i][k];
    }
    const NavCelId from = parent[i];
    if (surf[from] != NavMap::kNoSurface) {
      nm.surfaces[surf[from]].navcels.push_back(static_cast<NavCelId>(i));
    }
    if (first[from] == kNone) {first[from] = static_cast<NavCelId>(i);}
  }
  for (const auto & name : nm.list_layers()) {
    with_view(
      nm.layers.get(name), [&](auto & v) {
        auto & d = v.data();
        if (d.size() != old_n) {return;}
        std::remove_reference_t<decltype(d)> out(n);
        for (std::size_t i = 0; i < n; ++i) {
          out[i] = d[parent[i]];
        }
        d.swap(out);
        v.mark_all_changed();
      });
  }
  const auto links = nm.links();
  nm.clear_links();
  for (const auto & l : links) {
    if (l.from < old_n && l.to < old_n && first[l.from] != kNone && first[l.to] != kNone) {
      nm.add_link(first[l.from], first[l.to], l.type, l.length, l.bidirectional);
    }
  }
  Positions pos;
  pos.x.reserve(verts.p.size());
  pos.y.reserve(verts.p.size());
  pos.z.reserve(verts.p.size());
  for (const auto & q : verts.p) {
    pos.x.push_back(q.x());
    pos.y.push_back(q.y());
    pos.z.push_back(q.z());
  }
  nm.positions = std::move(pos);
  if (verts.colored) {
    Colors col;
    for (const auto & c : verts.c) {
      col.r.push_back(c[0]);
      col.g.push_back(c[1]);
      col.b.push_back(c[2]);
      col.a.push_back(c[3]);
    }
    nm.colors = std::move(col);
  }
  nm.navcels = std::move(cels);
  nm.rebuild_geometry_accels();
  st.navcels_after = nm.navcels.size();
  st.vertices_after = nm.positions.size();
  return st;
}

SubdivideStats stats_before(const NavMap & nm)
{
  SubdivideStats st;
  st.navcels_before = st.navcels_after = nm.navcels.size();
  st.vertices_before = st.vertices_after = nm.positions.size();
  return st;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// cut_along_lines
// ---------------------------------------------------------------------------------------------

SubdivideStats cut_along_lines(NavMap & nm, const CutLines & lines, float snap)
{
  SubdivideStats st = stats_before(nm);
  auto sorted = [](std::vector<float> v) {
      v.erase(
        std::remove_if(v.begin(), v.end(), [](float x) {return !std::isfinite(x);}), v.end());
      std::sort(v.begin(), v.end());
      v.erase(std::unique(v.begin(), v.end()), v.end());
      return v;
    };
  const std::vector<float> xs = sorted(lines.xs), ys = sorted(lines.ys);
  if (nm.navcels.empty() || (xs.empty() && ys.empty())) {return st;}

  Vertices verts(nm);
  // Vertices (almost) on a line go exactly onto it
  auto snap_axis = [&](int axis, const std::vector<float> & ls) {
      for (auto & q : verts.p) {
        auto it = std::lower_bound(ls.begin(), ls.end(), q[axis] - snap);
        if (it != ls.end() && std::abs(*it - q[axis]) <= snap) {q[axis] = *it;}
      }
    };
  snap_axis(0, xs);
  snap_axis(1, ys);

  // Line l: axis 0 (x = xs[l]) for l < xs.size(), else axis 1 (y = ys[l - xs.size()])
  const std::size_t nx = xs.size();
  auto axis_of = [nx](std::size_t l) {return l < nx ? 0 : 1;};
  auto value_of = [&](std::size_t l) {return l < nx ? xs[l] : ys[l - nx];};
  // The vertex where the edge (a, b) crosses line l, the same for the NavCels on both sides
  std::vector<std::unordered_map<uint64_t, uint32_t>> crossing(nx + ys.size());
  auto cross = [&](uint32_t a, uint32_t b, std::size_t l) {
      const uint64_t key = edge_key(a, b);
      auto it = crossing[l].find(key);
      if (it != crossing[l].end()) {return it->second;}
      const uint32_t u = std::min(a, b), w = std::max(a, b);
      const int axis = axis_of(l);
      const float c = value_of(l);
      const float t = (c - verts.p[u][axis]) / (verts.p[w][axis] - verts.p[u][axis]);
      const uint32_t m = verts.add_between(u, w, t);
      verts.p[m][axis] = c;
      crossing[l].emplace(key, m);
      return m;
    };

  std::vector<Tri> tris;
  std::vector<NavCelId> parent;
  tris.reserve(nm.navcels.size());
  parent.reserve(nm.navcels.size());
  std::vector<std::vector<uint32_t>> polys, next;
  std::vector<std::size_t> crossing_lines;
  for (NavCelId t = 0; t < nm.navcels.size(); ++t) {
    const auto & v = nm.navcels[t].v;
    // The lines strictly inside the NavCel's bounds, in a fixed order (x lines, then y lines,
    // each increasing): the NavCels sharing an edge split it the same way
    crossing_lines.clear();
    for (int axis = 0; axis < 2; ++axis) {
      float lo = std::numeric_limits<float>::infinity(), hi = -lo;
      for (int k = 0; k < 3; ++k) {
        lo = std::min(lo, verts.p[v[k]][axis]);
        hi = std::max(hi, verts.p[v[k]][axis]);
      }
      const auto & ls = axis == 0 ? xs : ys;
      for (auto it = std::upper_bound(ls.begin(), ls.end(), lo); it != ls.end() && *it < hi; ++it) {
        crossing_lines.push_back(static_cast<std::size_t>(it - ls.begin()) + (axis == 0 ? 0 : nx));
      }
    }
    if (crossing_lines.empty()) {
      tris.push_back({v[0], v[1], v[2]});
      parent.push_back(t);
      continue;
    }
    polys.assign(1, {v[0], v[1], v[2]});
    for (const std::size_t l : crossing_lines) {
      const int axis = axis_of(l);
      const float c = value_of(l);
      next.clear();
      for (const auto & poly : polys) {
        bool neg = false, pos = false;
        for (const uint32_t x : poly) {
          neg |= verts.p[x][axis] < c;
          pos |= verts.p[x][axis] > c;
        }
        if (!neg || !pos) {next.push_back(poly); continue;}
        std::vector<uint32_t> below, above;
        for (std::size_t i = 0; i < poly.size(); ++i) {
          const uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
          const float sa = verts.p[a][axis] - c, sb = verts.p[b][axis] - c;
          if (sa <= 0.0f) {below.push_back(a);}
          if (sa >= 0.0f) {above.push_back(a);}
          if ((sa < 0.0f && sb > 0.0f) || (sa > 0.0f && sb < 0.0f)) {
            const uint32_t m = cross(a, b, l);
            below.push_back(m);
            above.push_back(m);
          }
        }
        if (below.size() >= 3) {next.push_back(std::move(below));}
        if (above.size() >= 3) {next.push_back(std::move(above));}
      }
      polys.swap(next);
    }
    // The pieces are convex: a fan from their first vertex
    for (const auto & poly : polys) {
      for (std::size_t i = 1; i + 1 < poly.size(); ++i) {
        tris.push_back({poly[0], poly[i], poly[i + 1]});
        parent.push_back(t);
      }
    }
  }
  return replace(nm, tris, parent, verts, st);
}

// ---------------------------------------------------------------------------------------------
// refine
// ---------------------------------------------------------------------------------------------

namespace
{

class Refiner
{
public:
  Refiner(NavMap & nm, float max_edge)
  : nm_(nm), verts_(nm), max2_(static_cast<double>(max_edge) * max_edge)
  {
    const std::size_t n = nm.navcels.size();
    edges_.reserve(n * 2);
    tris_.resize(n);
    parent_.resize(n);
    fixed_.assign(n, 0);
    kids_.assign(n, {});
    for (NavCelId t = 0; t < n; ++t) {
      for (int k = 0; k < 3; ++k) {
        tris_[t][k] = nm.navcels[t].v[k];
      }
      parent_[t] = t;
      for (int k = 0; k < 3; ++k) {
        edges_[edge_key(tris_[t][k], tris_[t][(k + 1) % 3])].push_back(t);
      }
    }
    for (const auto & l : nm.links()) {
      for (const NavCelId c : {l.from, l.to}) {
        if (c < n) {fixed_[c] = 1;}
      }
    }
  }

  // Splits around q, starting from the NavCel t that holds it; a point on an edge or a vertex is
  // in the NavCels on the other side too, which are refined as well
  void around(const Eigen::Vector3f & q, uint32_t t)
  {
    std::vector<uint32_t> todo{holder(q, t)}, done;
    while (!todo.empty()) {
      uint32_t c = todo.back();
      todo.pop_back();
      if (std::find(done.begin(), done.end(), c) != done.end()) {continue;}
      c = refine_one(q, c);
      done.push_back(c);
      const auto b = bary(q, c);
      for (int k = 0; k < 3; ++k) {
        if (std::abs(b[(k + 2) % 3]) > 1e-5f) {continue;}   // not on the edge (k, k + 1)
        for (const uint32_t o : edges_[edge_key(tris_[c][k], tris_[c][(k + 1) % 3])]) {
          if (o != c && std::find(done.begin(), done.end(), o) == done.end()) {todo.push_back(o);}
        }
      }
    }
  }

  SubdivideStats finish(SubdivideStats st)
  {
    return replace(nm_, tris_, parent_, verts_, st);
  }

private:
  // Refines the NavCel t that holds q until it is small enough; the NavCel holding q then
  uint32_t refine_one(const Eigen::Vector3f & q, uint32_t t)
  {
    for (int guard = 0; guard < 100000; ++guard) {
      if (length2(longest(t)) <= max2_) {return t;}
      if (!split_edge(terminal_edge(t))) {return t;}
      t = holder(q, t);
    }
    return t;
  }

  // Following the longest edges from t, the first that is the longest of the NavCels on both
  // sides (or has no other side): the edge to split for t (0 if none is found)
  uint64_t terminal_edge(uint32_t t)
  {
    uint32_t cur = t;
    for (int depth = 0; depth < 10000; ++depth) {
      const uint64_t e = longest(cur);
      const auto & sides = edges_[e];
      uint32_t other = kNone;
      for (const uint32_t s : sides) {
        if (s != cur) {other = s;}
      }
      if (other == kNone || sides.size() > 2 || longest(other) == e) {return e;}
      cur = other;
    }
    return 0;
  }

public:
  // Splits until no NavCel has an edge longer than size(its vertices), starting with `todo`
  void where(const EdgeSizeAt & size, std::vector<uint32_t> todo)
  {
    std::vector<uint32_t> sides;
    for (std::size_t guard = 0; !todo.empty() && guard < 100000000; ++guard) {
      const uint32_t t = todo.back();
      todo.pop_back();
      const auto & v = tris_[t];
      const float m = size(verts_.p[v[0]], verts_.p[v[1]], verts_.p[v[2]]);
      if (!std::isfinite(m) || m <= 0.0f) {continue;}
      if (length2(longest(t)) <= static_cast<double>(m) * m) {continue;}
      const uint64_t e = terminal_edge(t);
      if (e == 0) {continue;}
      sides = edges_[e];
      if (!split_edge(e)) {continue;}
      // the NavCels split (and t, if the split was elsewhere) are looked at again
      todo.push_back(t);
      for (const uint32_t s : sides) {
        todo.push_back(s);
        todo.insert(todo.end(), kids_[s].end() - 1, kids_[s].end());
      }
    }
  }

private:
  double length2(uint64_t e) const
  {
    const uint32_t a = static_cast<uint32_t>(e >> 32), b = static_cast<uint32_t>(e);
    return static_cast<double>((verts_.p[a] - verts_.p[b]).squaredNorm());
  }

  // The longest edge of t; ties go to the smaller key, so both NavCels of an edge agree
  uint64_t longest(uint32_t t) const
  {
    uint64_t best = 0;
    double best_len = -1.0;
    for (int k = 0; k < 3; ++k) {
      const uint64_t e = edge_key(tris_[t][k], tris_[t][(k + 1) % 3]);
      const double len = length2(e);
      if (len > best_len || (len == best_len && e < best)) {best = e; best_len = len;}
    }
    return best;
  }

  void unlink(uint64_t e, uint32_t t)
  {
    auto & v = edges_[e];
    v.erase(std::remove(v.begin(), v.end(), t), v.end());
  }

  // Splits the edge e at its middle in every NavCel that has it
  bool split_edge(uint64_t e)
  {
    const std::vector<uint32_t> sides = edges_[e];
    for (const uint32_t t : sides) {
      if (fixed_[t]) {return false;}
    }
    const uint32_t a = static_cast<uint32_t>(e >> 32), b = static_cast<uint32_t>(e);
    const uint32_t m = verts_.add_between(a, b, 0.5f);
    for (const uint32_t t : sides) {
      // t = (p, q, o) with the edge (p, q) in its winding: t -> (p, m, o), new -> (m, q, o)
      int k = 0;
      while (edge_key(tris_[t][k], tris_[t][(k + 1) % 3]) != e) {++k;}
      const uint32_t p = tris_[t][k], q = tris_[t][(k + 1) % 3], o = tris_[t][(k + 2) % 3];
      const uint32_t t2 = static_cast<uint32_t>(tris_.size());
      tris_[t] = {p, m, o};
      tris_.push_back({m, q, o});
      parent_.push_back(parent_[t]);
      fixed_.push_back(0);
      kids_[t].push_back(t2);
      kids_.emplace_back();
      unlink(edge_key(q, o), t);
      edges_[edge_key(q, o)].push_back(t2);
      edges_[edge_key(p, m)].push_back(t);
      edges_[edge_key(m, q)].push_back(t2);
      edges_[edge_key(m, o)].push_back(t);
      edges_[edge_key(m, o)].push_back(t2);
    }
    edges_.erase(e);
    return true;
  }

  // The NavCel holding q now: t, or one of the NavCels split from it since (the most inside)
  uint32_t holder(const Eigen::Vector3f & q, uint32_t t) const
  {
    uint32_t best = t;
    float best_in = inside(q, t);
    std::vector<uint32_t> stack(kids_[t].begin(), kids_[t].end());
    while (!stack.empty()) {
      const uint32_t k = stack.back();
      stack.pop_back();
      const float in = inside(q, k);
      if (in > best_in) {best = k; best_in = in;}
      stack.insert(stack.end(), kids_[k].begin(), kids_[k].end());
    }
    return best;
  }

  // Barycentric coordinates of q in t, in the plane (NaN for a NavCel with no area there)
  std::array<float, 3> bary(const Eigen::Vector3f & q, uint32_t t) const
  {
    const Eigen::Vector2f a = verts_.p[tris_[t][0]].head<2>(), b = verts_.p[tris_[t][1]].head<2>(),
      c = verts_.p[tris_[t][2]].head<2>(), x = q.head<2>();
    const float d = (b.x() - a.x()) * (c.y() - a.y()) - (c.x() - a.x()) * (b.y() - a.y());
    if (d == 0.0f) {
      const float nan = std::numeric_limits<float>::quiet_NaN();
      return {nan, nan, nan};
    }
    const float u = ((b.x() - x.x()) * (c.y() - x.y()) - (c.x() - x.x()) * (b.y() - x.y())) / d;
    const float v = ((c.x() - x.x()) * (a.y() - x.y()) - (a.x() - x.x()) * (c.y() - x.y())) / d;
    return {u, v, 1.0f - u - v};
  }

  // How far inside t q is, in the plane: the smallest barycentric coordinate
  float inside(const Eigen::Vector3f & q, uint32_t t) const
  {
    const auto b = bary(q, t);
    if (std::isnan(b[0])) {return -std::numeric_limits<float>::infinity();}
    return std::min({b[0], b[1], b[2]});
  }

  NavMap & nm_;
  Vertices verts_;
  const double max2_;
  std::vector<Tri> tris_;
  std::vector<NavCelId> parent_;
  std::vector<char> fixed_;
  std::unordered_map<uint64_t, std::vector<uint32_t>> edges_;
  std::vector<std::vector<uint32_t>> kids_;  // the NavCels split from each one
};

}  // namespace

SubdivideStats refine(NavMap & nm, const EdgeSizeAt & max_edge_at)
{
  SubdivideStats st = stats_before(nm);
  if (nm.navcels.empty() || !max_edge_at) {return st;}
  // The NavCels too large: if none, the map is left as it is
  std::vector<uint32_t> todo;
  for (NavCelId c = 0; c < nm.navcels.size(); ++c) {
    const auto & v = nm.navcels[c].v;
    const Eigen::Vector3f a = nm.positions.at(v[0]), b = nm.positions.at(v[1]),
      d = nm.positions.at(v[2]);
    const float m = max_edge_at(a, b, d);
    if (!std::isfinite(m) || m <= 0.0f) {continue;}
    const float longest2 = std::max({(a - b).squaredNorm(), (b - d).squaredNorm(),
          (d - a).squaredNorm()});
    if (longest2 > m * m) {todo.push_back(c);}
  }
  if (todo.empty()) {return st;}
  std::reverse(todo.begin(), todo.end());
  Refiner r(nm, 0.0f);
  r.where(max_edge_at, std::move(todo));
  return r.finish(st);
}

SubdivideStats refine(NavMap & nm, const std::vector<Eigen::Vector3f> & points, float max_edge)
{
  SubdivideStats st = stats_before(nm);
  if (nm.navcels.empty() || points.empty() || !(max_edge > 0.0f)) {return st;}
  // Where the points are, on the map as it is
  std::vector<std::pair<Eigen::Vector3f, uint32_t>> start;
  for (const auto & q : points) {
    std::size_t s;
    NavCelId cid;
    Eigen::Vector3f bary;
    if (nm.locate_navcel(q, s, cid, bary, nullptr)) {start.emplace_back(q, cid);}
  }
  if (start.empty()) {return st;}
  Refiner r(nm, max_edge);
  for (const auto & [q, cid] : start) {
    r.around(q, cid);
  }
  return r.finish(st);
}

}  // namespace navmap
