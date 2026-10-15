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

#include "navmap_core/Simplify.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <queue>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "navmap_core/Geometry.hpp"

namespace navmap
{

namespace
{

using Vec3d = Eigen::Vector3d;
using Quadric = Eigen::Matrix4d;
constexpr uint32_t kNone = std::numeric_limits<uint32_t>::max();

// Calls f(view) with the typed view of @p base (no-op for an unknown type).
template<class F>
void with_view(const std::shared_ptr<LayerViewBase> & base, F && f)
{
  if (auto v = std::dynamic_pointer_cast<LayerView<uint8_t>>(base)) {f(*v); return;}
  if (auto v = std::dynamic_pointer_cast<LayerView<float>>(base)) {f(*v); return;}
  if (auto v = std::dynamic_pointer_cast<LayerView<double>>(base)) {f(*v); return;}
  if (auto v = std::dynamic_pointer_cast<LayerView<int16_t>>(base)) {f(*v); return;}
}

Quadric plane_quadric(const Vec3d & n, const Vec3d & p)
{
  const Eigen::Vector4d q(n.x(), n.y(), n.z(), -n.dot(p));
  return q * q.transpose();
}

class Simplifier
{
public:
  Simplifier(NavMap & nm, const SimplifyParams & p)
  : nm_(nm), p_(p), cos_normal_(std::cos(p.max_normal_change * M_PI / 180.0)),
    max_err2_(static_cast<double>(p.max_error) * p.max_error)
  {}

  SimplifyStats run()
  {
    SimplifyStats st;
    st.navcels_before = nm_.navcels.size();
    st.vertices_before = nm_.positions.size();
    if (nm_.navcels.empty()) {return st;}
    init();
    for (uint32_t u = 0; u < P_.size(); ++u) {
      push_best(u);
    }
    while (!pq_.empty()) {
      const Entry e = pq_.top();
      pq_.pop();
      if (dead_[e.u] || e.stamp != stamp_[e.u]) {continue;}
      double cost;
      if (!valid(e.u, e.v, cost)) {push_best(e.u); continue;}
      collapse(e.u, e.v);
      for (const uint32_t x : neighbors(e.v)) {
        push_best(x);
      }
      push_best(e.v);
    }
    compact();
    st.navcels_after = nm_.navcels.size();
    st.vertices_after = nm_.positions.size();
    return st;
  }

private:
  struct Entry
  {
    double cost;
    uint32_t u, v, stamp;
    bool operator<(const Entry & o) const {return cost > o.cost;}  // min-heap
  };

  void init()
  {
    const std::size_t nv = nm_.positions.size(), nt = nm_.navcels.size();
    P_.resize(nv);
    for (uint32_t i = 0; i < nv; ++i) {
      P_[i] = nm_.positions.at(i).cast<double>();
    }
    T_.resize(nt);
    N0_.resize(nt);
    alive_.assign(nt, 1);
    VT_.assign(nv, {});
    Q_.assign(nv, Quadric::Zero());
    dead_.assign(nv, 0);
    locked_.assign(nv, 0);
    stamp_.assign(nv, 0);
    for (uint32_t t = 0; t < nt; ++t) {
      for (int k = 0; k < 3; ++k) {
        T_[t][k] = nm_.navcels[t].v[k];
        VT_[T_[t][k]].push_back(t);
      }
      const Vec3d a = P_[T_[t][0]], b = P_[T_[t][1]], c = P_[T_[t][2]];
      const Vec3d n = (b - a).cross(c - a);
      N0_[t] = n.norm() > 0.0 ? Vec3d(n.normalized()) : Vec3d::UnitZ();
      const Quadric q = plane_quadric(N0_[t], a);
      for (int k = 0; k < 3; ++k) {
        Q_[T_[t][k]] += q;
      }
    }
    // Open edges: a plane through the edge, perpendicular to its NavCel, keeps the border's shape.
    // Edges of 3+ NavCels (non-manifold) lock their vertices.
    for (uint32_t t = 0; t < nt; ++t) {
      for (int k = 0; k < 3; ++k) {
        const uint32_t a = T_[t][k], b = T_[t][(k + 1) % 3];
        const std::size_t m = edge_tris(a, b).size();
        if (m > 2) {locked_[a] = locked_[b] = 1;}
        if (m != 1) {continue;}
        const Vec3d e = P_[b] - P_[a];
        const Vec3d n = e.cross(N0_[t]);
        if (n.norm() == 0.0) {continue;}
        const Quadric q = plane_quadric(n.normalized(), P_[a]);
        Q_[a] += q;
        Q_[b] += q;
      }
    }
    // Same values in every kept layer <=> same class
    std::vector<std::string> names = p_.layers.empty() ? nm_.list_layers() : p_.layers;
    std::vector<std::vector<double>> cols;
    for (const auto & name : names) {
      if (std::find(p_.ignore_layers.begin(), p_.ignore_layers.end(), name) !=
        p_.ignore_layers.end())
      {
        continue;
      }
      const auto base = nm_.layers.get(name);
      if (!base || base->size() != nt) {continue;}
      std::vector<double> col(nt);
      with_view(
        base, [&](const auto & v) {
          for (std::size_t c = 0; c < nt; ++c) {
            col[c] = static_cast<double>(v.data()[c]);
          }
        });
      cols.push_back(std::move(col));
    }
    cls_.assign(nt, 0);
    std::unordered_map<std::string, uint32_t> ids;
    std::string key(cols.size() * sizeof(double), '\0');
    for (std::size_t t = 0; t < nt; ++t) {
      for (std::size_t l = 0; l < cols.size(); ++l) {
        double v = cols[l][t];
        if (std::isnan(v)) {v = std::numeric_limits<double>::quiet_NaN();}  // one NaN
        if (v == 0.0) {v = 0.0;}  // one zero
        std::memcpy(key.data() + l * sizeof(double), &v, sizeof(double));
      }
      cls_[t] = ids.try_emplace(key, static_cast<uint32_t>(ids.size())).first->second;
    }
    surf_.assign(nt, NavMap::kNoSurface);
    for (std::size_t s = 0; s < nm_.surfaces.size(); ++s) {
      for (const NavCelId c : nm_.surfaces[s].navcels) {
        if (c < nt) {surf_[c] = s;}
      }
    }
    // NavCels at the end of a link must survive as they are
    for (const auto & l : nm_.links()) {
      for (const NavCelId c : {l.from, l.to}) {
        if (c >= nt) {continue;}
        for (const uint32_t x : T_[c]) {
          locked_[x] = 1;
        }
      }
    }
    for (uint32_t u = 0; u < nv; ++u) {
      if (VT_[u].empty()) {dead_[u] = 1;}
    }
  }

  std::vector<uint32_t> edge_tris(uint32_t a, uint32_t b) const
  {
    std::vector<uint32_t> out;
    for (const uint32_t t : VT_[a]) {
      if (T_[t][0] == b || T_[t][1] == b || T_[t][2] == b) {out.push_back(t);}
    }
    return out;
  }

  std::vector<uint32_t> neighbors(uint32_t u) const
  {
    std::vector<uint32_t> out;
    for (const uint32_t t : VT_[u]) {
      for (const uint32_t x : T_[t]) {
        if (x != u) {out.push_back(x);}
      }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
  }

  // Whether collapsing u into v keeps layers, border, topology and quality; its quadric cost.
  bool valid(uint32_t u, uint32_t v, double & cost) const
  {
    if (locked_[u] || dead_[u] || dead_[v] || u == v) {return false;}
    const auto & fan = VT_[u];
    for (const uint32_t t : fan) {
      if (cls_[t] != cls_[fan[0]] || surf_[t] != surf_[fan[0]]) {return false;}
    }
    const auto et = edge_tris(u, v);
    if (et.empty() || et.size() > 2) {return false;}
    // Border: u on an open edge moves only along it, onto a neighbour on the same open edge
    const auto nu = neighbors(u);
    std::vector<uint32_t> open;
    for (const uint32_t w : nu) {
      if (edge_tris(u, w).size() == 1) {open.push_back(w);}
    }
    if (open.size() == 2) {
      if (et.size() != 1) {return false;}
      const uint32_t w = open[0] == v ? open[1] : (open[1] == v ? open[0] : kNone);
      if (w == kNone) {return false;}
      const Vec3d d = P_[w] - P_[v];
      const double len2 = d.squaredNorm();
      if (len2 == 0.0) {return false;}
      const double s = (P_[u] - P_[v]).dot(d) / len2;
      if (s <= 0.0 || s >= 1.0) {return false;}
      if ((P_[v] + s * d - P_[u]).squaredNorm() > max_err2_) {return false;}
    } else if (!open.empty() || et.size() != 2) {
      return false;
    }
    // Link condition: u and v share exactly the opposite vertices of their common NavCels
    const auto nv = neighbors(v);
    std::size_t common = 0;
    for (const uint32_t x : nu) {
      if (std::binary_search(nv.begin(), nv.end(), x)) {++common;}
    }
    if (common != et.size()) {return false;}
    // The NavCels that stay, with v instead of u
    const double max_edge2 = static_cast<double>(p_.max_edge_length) * p_.max_edge_length;
    bool any = false;
    double dev2 = std::numeric_limits<double>::infinity();
    for (const uint32_t t : fan) {
      if (std::find(et.begin(), et.end(), t) != et.end()) {continue;}
      std::array<Vec3d, 3> q;
      for (int k = 0; k < 3; ++k) {
        q[k] = T_[t][k] == u ? P_[v] : P_[T_[t][k]];
      }
      const Vec3d n = (q[1] - q[0]).cross(q[2] - q[0]);
      const double a2 = n.norm();
      const double sum = (q[1] - q[0]).squaredNorm() + (q[2] - q[1]).squaredNorm() +
        (q[0] - q[2]).squaredNorm();
      if (a2 <= 1e-12 * sum) {return false;}
      if (n.dot(N0_[t]) / a2 < cos_normal_) {return false;}
      if (2.0 * std::sqrt(3.0) * a2 / sum < p_.min_quality) {return false;}
      for (int k = 0; k < 3; ++k) {
        if (T_[t][k] != u && (P_[T_[t][k]] - P_[v]).squaredNorm() > max_edge2) {return false;}
      }
      dev2 = std::min(
        dev2, static_cast<double>(point_triangle_squared_distance(
          P_[u].cast<float>(), q[0].cast<float>(), q[1].cast<float>(), q[2].cast<float>())));
      any = true;
    }
    if (!any || dev2 > max_err2_) {return false;}
    const Eigen::Vector4d h(P_[v].x(), P_[v].y(), P_[v].z(), 1.0);
    cost = std::max(0.0, h.dot((Q_[u] + Q_[v]) * h));
    return cost <= max_err2_;
  }

  void push_best(uint32_t u)
  {
    ++stamp_[u];
    if (dead_[u] || locked_[u]) {return;}
    double best = std::numeric_limits<double>::infinity();
    uint32_t to = kNone;
    for (const uint32_t v : neighbors(u)) {
      double c;
      if (valid(u, v, c) && c < best) {best = c; to = v;}
    }
    if (to != kNone) {pq_.push({best, u, to, stamp_[u]});}
  }

  void collapse(uint32_t u, uint32_t v)
  {
    for (const uint32_t t : edge_tris(u, v)) {
      alive_[t] = 0;
      for (const uint32_t x : T_[t]) {
        auto & l = VT_[x];
        l.erase(std::remove(l.begin(), l.end(), t), l.end());
      }
    }
    for (const uint32_t t : VT_[u]) {
      for (auto & x : T_[t]) {
        if (x == u) {x = v;}
      }
      VT_[v].push_back(t);
    }
    VT_[u].clear();
    Q_[v] += Q_[u];
    dead_[u] = 1;
  }

  void compact()
  {
    const std::size_t nt = T_.size();
    std::vector<uint32_t> vmap(P_.size(), kNone), tmap(nt, kNone);
    std::vector<NavCelId> kept;
    Positions pos;
    std::optional<Colors> col;
    const bool has_col = nm_.colors && nm_.colors->r.size() == P_.size();
    if (has_col) {col.emplace();}
    std::vector<NavCel> cels;
    for (uint32_t t = 0; t < nt; ++t) {
      if (!alive_[t]) {continue;}
      NavCel c;
      for (int k = 0; k < 3; ++k) {
        const uint32_t x = T_[t][k];
        if (vmap[x] == kNone) {
          vmap[x] = static_cast<uint32_t>(pos.size());
          pos.x.push_back(static_cast<float>(P_[x].x()));
          pos.y.push_back(static_cast<float>(P_[x].y()));
          pos.z.push_back(static_cast<float>(P_[x].z()));
          if (has_col) {
            col->r.push_back(nm_.colors->r[x]);
            col->g.push_back(nm_.colors->g[x]);
            col->b.push_back(nm_.colors->b[x]);
            col->a.push_back(nm_.colors->a[x]);
          }
        }
        c.v[k] = vmap[x];
      }
      tmap[t] = static_cast<uint32_t>(cels.size());
      kept.push_back(t);
      cels.push_back(c);
    }
    for (auto & s : nm_.surfaces) {
      std::vector<NavCelId> ids;
      for (const NavCelId c : s.navcels) {
        if (c < nt && tmap[c] != kNone) {ids.push_back(tmap[c]);}
      }
      s.navcels = std::move(ids);
    }
    for (const auto & name : nm_.list_layers()) {
      const auto base = nm_.layers.get(name);
      with_view(
        base, [&](auto & v) {
          auto & d = v.data();
          std::remove_reference_t<decltype(d)> out(kept.size());
          for (std::size_t i = 0; i < kept.size(); ++i) {
            if (kept[i] < d.size()) {out[i] = d[kept[i]];}
          }
          d.swap(out);
          v.mark_all_changed();
        });
    }
    const auto links = nm_.links();
    nm_.positions = std::move(pos);
    nm_.colors = std::move(col);
    nm_.navcels = std::move(cels);
    nm_.clear_links();
    for (const auto & l : links) {
      if (l.from < nt && l.to < nt && tmap[l.from] != kNone && tmap[l.to] != kNone) {
        nm_.add_link(tmap[l.from], tmap[l.to], l.type, l.length, l.bidirectional);
      }
    }
    nm_.rebuild_geometry_accels();
  }

  NavMap & nm_;
  const SimplifyParams & p_;
  const double cos_normal_, max_err2_;
  std::vector<Vec3d> P_;
  std::vector<std::array<uint32_t, 3>> T_;
  std::vector<Vec3d> N0_;
  std::vector<char> alive_, dead_, locked_;
  std::vector<std::vector<uint32_t>> VT_;
  std::vector<Quadric> Q_;
  std::vector<uint32_t> cls_, stamp_;
  std::vector<std::size_t> surf_;
  std::priority_queue<Entry> pq_;
};

}  // namespace

SimplifyStats simplify(NavMap & nm, const SimplifyParams & params)
{
  return Simplifier(nm, params).run();
}

}  // namespace navmap
