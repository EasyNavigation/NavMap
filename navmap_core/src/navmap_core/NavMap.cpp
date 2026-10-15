// Copyright 2025 Intelligent Robotics Lab
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


#include "navmap_core/NavMap.hpp"
#include <array>
#include <queue>
#include <vector>
#include <algorithm>
#include <functional>
#include <cmath>
#include <limits>
#include <cstdint>
#include <unordered_set>

namespace navmap
{


namespace
{
// 64-bit FNV-1a hash
inline std::uint64_t fnv1a64(
  const void * data, std::size_t n,
  std::uint64_t seed = 1469598103934665603ULL)
{
  const auto * p = static_cast<const std::uint8_t *>(data);
  std::uint64_t h = seed;
  for (std::size_t i = 0; i < n; ++i) {
    h ^= p[i];
    h *= 1099511628211ULL;
  }
  return h;
}
}  // namespace

NavMap::NavMap()
{
}

NavMap::NavMap(const NavMap & other)
{
  positions = other.positions;
  colors = other.colors;
  navcels = other.navcels;
  surfaces = other.surfaces;
  layers = other.layers;
  layer_meta = other.layer_meta;
  navcel_surface_ = other.navcel_surface_;
  nonmanifold_edges_ = other.nonmanifold_edges_;

  geometry_dirty_ = true;
  geometry_fp_ = 0;
}

void NavMap::ensure_geometry_fingerprint_() const
{
  if (!geometry_dirty_) {return;}

  const std::size_t nx = positions.x.size();
  const std::size_t ny = positions.y.size();
  const std::size_t nz = positions.z.size();

  std::uint64_t h = 1469598103934665603ULL;
  h = fnv1a64(&nx, sizeof(nx), h); h = fnv1a64(positions.x.data(), nx * sizeof(float), h);
  h = fnv1a64(&ny, sizeof(ny), h); h = fnv1a64(positions.y.data(), ny * sizeof(float), h);
  h = fnv1a64(&nz, sizeof(nz), h); h = fnv1a64(positions.z.data(), nz * sizeof(float), h);

  const std::size_t nt = navcels.size();
  h = fnv1a64(&nt, sizeof(nt), h);
  for (const auto & c : navcels) {
    h = fnv1a64(c.v, sizeof(c.v), h);
  }

  geometry_fp_ = h;
  geometry_dirty_ = false;
}

std::uint64_t NavMap::hash_geometry_bytes_(
  const float * x, std::size_t nx,
  const float * y, std::size_t ny,
  const float * z, std::size_t nz,
  const std::uint32_t * v0, std::size_t nv0,
  const std::uint32_t * v1, std::size_t nv1,
  const std::uint32_t * v2, std::size_t nv2)
{
  std::uint64_t h = 1469598103934665603ULL;
  h = fnv1a64(&nx, sizeof(nx), h); h = fnv1a64(x, nx * sizeof(float), h);
  h = fnv1a64(&ny, sizeof(ny), h); h = fnv1a64(y, ny * sizeof(float), h);
  h = fnv1a64(&nz, sizeof(nz), h); h = fnv1a64(z, nz * sizeof(float), h);
  h = fnv1a64(&nv0, sizeof(nv0), h); h = fnv1a64(v0, nv0 * sizeof(std::uint32_t), h);
  h = fnv1a64(&nv1, sizeof(nv1), h); h = fnv1a64(v1, nv1 * sizeof(std::uint32_t), h);
  h = fnv1a64(&nv2, sizeof(nv2), h); h = fnv1a64(v2, nv2 * sizeof(std::uint32_t), h);
  return h;
}

bool NavMap::has_same_geometry(const NavMap & other) const
{
  if (positions.x.size() != other.positions.x.size() ||
    positions.y.size() != other.positions.y.size() ||
    positions.z.size() != other.positions.z.size() ||
    navcels.size() != other.navcels.size())
  {
    return false;
  }

  ensure_geometry_fingerprint_();
  other.ensure_geometry_fingerprint_();
  return geometry_fp_ == other.geometry_fp_;
}


NavMap & NavMap::operator=(const NavMap & other)
{
  if (this == &other) {return *this;}

  if (has_same_geometry(other)) {
    // Remove destination-only layers to make maps identical.
    {
      auto src_names = other.layers.list();
      std::unordered_set<std::string> wanted(src_names.begin(), src_names.end());
      for (const auto & name : layers.list()) {
        if (!wanted.count(name)) {
          layers.remove(name);
        }
      }
    }

    // Synchronize layers using cached content hashes
    for (const auto & name : other.layers.list()) {
      auto src_base = other.layers.get(name);
      if (!src_base) {continue;}

      switch (src_base->type()) {
        case LayerType::U8: {
            auto src = std::dynamic_pointer_cast<LayerView<uint8_t>>(src_base);
            auto dst = layers.add_or_get<uint8_t>(name, navcels.size(), layer_type_tag<uint8_t>());
            const bool same_size = dst->data().size() == src->data().size();
            const bool same_hash = same_size && (dst->content_hash() == src->content_hash());
            if (!same_hash) {dst->set_data(src->data());}
            break;
          }
        case LayerType::F32: {
            auto src = std::dynamic_pointer_cast<LayerView<float>>(src_base);
            auto dst = layers.add_or_get<float>(name, navcels.size(), layer_type_tag<float>());
            const bool same_size = dst->data().size() == src->data().size();
            const bool same_hash = same_size && (dst->content_hash() == src->content_hash());
            if (!same_hash) {dst->set_data(src->data());}
            break;
          }
        case LayerType::F64: {
            auto src = std::dynamic_pointer_cast<LayerView<double>>(src_base);
            auto dst = layers.add_or_get<double>(name, navcels.size(), layer_type_tag<double>());
            const bool same_size = dst->data().size() == src->data().size();
            const bool same_hash = same_size && (dst->content_hash() == src->content_hash());
            if (!same_hash) {dst->set_data(src->data());}
            break;
          }
        default: break;
      }
    }

    layer_meta = other.layer_meta;
    return *this;
  }

  // Full deep copy
  positions = other.positions;
  colors = other.colors;
  navcels = other.navcels;
  surfaces = other.surfaces;
  layers = other.layers;
  layer_meta = other.layer_meta;
  navcel_surface_ = other.navcel_surface_;
  nonmanifold_edges_ = other.nonmanifold_edges_;

  geometry_dirty_ = true;
  return *this;
}


NavMap & NavMap::operator=(NavMap && other) noexcept
{
  if (this == &other) {return *this;}

  positions = std::move(other.positions);
  colors = std::move(other.colors);
  navcels = std::move(other.navcels);
  surfaces = std::move(other.surfaces);
  layers = std::move(other.layers);
  layer_meta = std::move(other.layer_meta);
  navcel_surface_ = std::move(other.navcel_surface_);
  nonmanifold_edges_ = other.nonmanifold_edges_;

  geometry_fp_ = other.geometry_fp_;
  geometry_dirty_ = other.geometry_dirty_;
  other.geometry_dirty_ = true;
  other.geometry_fp_ = 0;
  return *this;
}


// -----------------------------------------------------------------------------
// Internal: utilities
// -----------------------------------------------------------------------------

static inline AABB triangle_aabb(
  const Vec3 & a,
  const Vec3 & b,
  const Vec3 & c)
{
  AABB box;
  box.expand(a);
  box.expand(b);
  box.expand(c);
  return box;
}

// -----------------------------------------------------------------------------
// Adjacency
// -----------------------------------------------------------------------------

void NavMap::build_adjacency()
{
  constexpr auto kNone = std::numeric_limits<NavCelId>::max();
  struct EdgeUse
  {
    PointId a, b;
    NavCelId cid;
    uint8_t e;
  };
  std::vector<EdgeUse> uses;
  uses.reserve(navcels.size() * 3);
  for (NavCelId cid = 0; cid < navcels.size(); ++cid) {
    auto & c = navcels[cid];
    for (uint8_t e = 0; e < 3; ++e) {
      c.neighbor[e] = kNone;
      PointId a = c.v[e];
      PointId b = c.v[(e + 1) % 3];
      if (a > b) {std::swap(a, b);}
      uses.push_back({a, b, cid, e});
    }
  }
  std::sort(
    uses.begin(), uses.end(), [](const EdgeUse & x, const EdgeUse & y) {
      return x.a != y.a ? x.a < y.a : x.b < y.b;
    });

  // Link only edges shared by exactly two cells; leave non-manifold edges open.
  nonmanifold_edges_ = 0;
  for (std::size_t k = 0; k < uses.size(); ) {
    std::size_t end = k + 1;
    while (end < uses.size() && uses[end].a == uses[k].a && uses[end].b == uses[k].b) {++end;}
    if (end - k == 2) {
      navcels[uses[k].cid].neighbor[uses[k].e] = uses[k + 1].cid;
      navcels[uses[k + 1].cid].neighbor[uses[k + 1].e] = uses[k].cid;
    } else if (end - k > 2) {
      ++nonmanifold_edges_;
    }
    k = end;
  }
}

// -----------------------------------------------------------------------------
// BVH build per surface
// -----------------------------------------------------------------------------

void NavMap::build_surface_bvh(Surface & s)
{
  s.prim_indices.clear();
  s.prim_indices.reserve(s.navcels.size());
  s.aabb = AABB{};

  for (auto cid : s.navcels) {
    const auto & c = navcels[cid];
    Vec3 a = positions.at(c.v[0]);
    Vec3 b = positions.at(c.v[1]);
    Vec3 d = positions.at(c.v[2]);
    s.aabb.expand(a);
    s.aabb.expand(b);
    s.aabb.expand(d);
    s.prim_indices.push_back(static_cast<int>(cid));
  }

  struct PrimBox
  {
    int cid;
    AABB box;
    Vec3 centroid;
  };
  std::vector<PrimBox> prims;
  prims.reserve(s.prim_indices.size());
  for (int cid : s.prim_indices) {
    const auto & c = navcels[cid];
    Vec3 a = positions.at(c.v[0]);
    Vec3 b = positions.at(c.v[1]);
    Vec3 d = positions.at(c.v[2]);
    AABB box = triangle_aabb(a, b, d);
    prims.push_back({cid, box, (a + b + d) / 3.0f});
  }

  s.bvh.clear();
  s.bvh.reserve(2 * prims.size());

  std::function<int(int, int)> build = [&](int begin, int end) -> int {
      BVHNode node;
      for (int i = begin; i < end; ++i) {
        node.box.expand(prims[i].box);
      }
      int idx = static_cast<int>(s.bvh.size());
      s.bvh.push_back(node);

      int count = end - begin;
      if (count <= 4) {
        s.bvh[idx].start = begin;
        s.bvh[idx].count = count;
        return idx;
      }

      int axis = s.bvh[idx].box.longest_axis();
      int mid = (begin + end) / 2;
      std::nth_element(
        prims.begin() + begin,
        prims.begin() + mid,
        prims.begin() + end,
        [&](const PrimBox & A, const PrimBox & B) {
          return A.centroid[axis] < B.centroid[axis];
        });
      int L = build(begin, mid);
      int R = build(mid, end);
      s.bvh[idx].left = L;
      s.bvh[idx].right = R;
      return idx;
    };

  if (!prims.empty()) {
    build(0, static_cast<int>(prims.size()));
  }

  s.prim_indices.clear();
  s.prim_indices.reserve(prims.size());
  for (auto & p : prims) {
    s.prim_indices.push_back(p.cid);
  }
}

// -----------------------------------------------------------------------------
// BVH traversal stack: fixed buffer, heap only if a degenerate tree overflows it
// -----------------------------------------------------------------------------

namespace
{
class NodeStack
{
public:
  void push(int v)
  {
    if (n_ < static_cast<int>(buf_.size())) {buf_[n_++] = v;} else {spill_.push_back(v);}
  }
  int pop()
  {
    if (!spill_.empty()) {
      const int v = spill_.back();
      spill_.pop_back();
      return v;
    }
    return buf_[--n_];
  }
  bool empty() const {return n_ == 0 && spill_.empty();}

private:
  std::array<int, 96> buf_;
  int n_{0};
  std::vector<int> spill_;
};

// Entry distance of the ray into the box, if it enters before t_max.
inline bool ray_box_entry(
  const AABB & box, const Vec3 & o, const Vec3 & d, float t_max,
  float & t_in)
{
  float t0 = 0.0f, t1 = t_max;
  for (int i = 0; i < 3; ++i) {
    if (std::fabs(d[i]) < 1e-12f) {
      if (o[i] < box.min[i] || o[i] > box.max[i]) {return false;}
      continue;
    }
    const float inv = 1.0f / d[i];
    float a = (box.min[i] - o[i]) * inv, b = (box.max[i] - o[i]) * inv;
    if (a > b) {std::swap(a, b);}
    t0 = std::max(t0, a);
    t1 = std::min(t1, b);
    if (!(t0 <= t1)) {return false;}  // also rejects NaN
  }
  t_in = t0;
  return true;
}

inline float box_sq_dist(const AABB & box, const Vec3 & p)
{
  const float dx = std::max(std::max(box.min.x() - p.x(), 0.0f), p.x() - box.max.x());
  const float dy = std::max(std::max(box.min.y() - p.y(), 0.0f), p.y() - box.max.y());
  const float dz = std::max(std::max(box.min.z() - p.z(), 0.0f), p.z() - box.max.z());
  return dx * dx + dy * dy + dz * dz;
}
}  // namespace

// -----------------------------------------------------------------------------
// Rebuild accelerations and geometry caches
// -----------------------------------------------------------------------------

void NavMap::rebuild_geometry_accels()
{
  // Normals and areas.
  for (auto & c : navcels) {
    Vec3 a = positions.at(c.v[0]);
    Vec3 b = positions.at(c.v[1]);
    Vec3 d = positions.at(c.v[2]);
    c.normal = triangle_normal(a, b, d);
    c.area = triangle_area(a, b, d);
  }

  // Topological neighbors.
  build_adjacency();

  // Per-surface BVH and seed grid.
  for (auto & s : surfaces) {
    build_surface_bvh(s);
  }
  build_navcel_surface_index_();

  // NOTE: Layers are per-NavCel; if navcels size changed, caller may
  // choose to call layers.resize_all(navcels.size()) externally.
}

// -----------------------------------------------------------------------------
// Raycast traversal
// -----------------------------------------------------------------------------

bool NavMap::surface_raycast(
  const Surface & s,
  const Vec3 & o,
  const Vec3 & d,
  NavCelId & hit_cid,
  float & t_out,
  Vec3 & hit_pt,
  float t_max) const
{
  if (s.bvh.empty()) {
    return false;
  }

  bool hit = false;
  float best_t = t_max;
  NodeStack st;
  st.push(0);

  while (!st.empty()) {
    const BVHNode & n = s.bvh[st.pop()];
    if (!n.box.intersects_ray(o, d, best_t)) {
      continue;
    }
    if (n.is_leaf()) {
      for (int i = 0; i < n.count; ++i) {
        int cid = s.prim_indices[n.start + i];
        const auto & c = navcels[cid];
        Vec3 a = positions.at(c.v[0]);
        Vec3 b = positions.at(c.v[1]);
        Vec3 e = positions.at(c.v[2]);
        float t, u, v;
        if (ray_triangle_intersect(o, d, a, b, e, t, u, v)) {
          if (t < best_t) {
            best_t = t;
            hit_cid = cid;
            hit_pt = o + t * d;
            hit = true;
          }
        }
      }
    } else {
      if (n.left >= 0) {st.push(n.left);}
      if (n.right >= 0) {st.push(n.right);}
    }
  }

  if (hit) {
    t_out = best_t;
  }
  return hit;
}

bool NavMap::raycast(
  const Vec3 & o,
  const Vec3 & d,
  NavCelId & hit_cid,
  float & t,
  Vec3 & hit_pt) const
{
  RayHit h;
  if (!closest_hit_(o, d, std::numeric_limits<float>::infinity(), h)) {
    return false;
  }
  hit_cid = h.cid;
  t = h.t;
  hit_pt = h.p;
  return true;
}

bool NavMap::closest_hit_(const Vec3 & o, const Vec3 & d, float t_max, RayHit & out) const
{
  // Surfaces the ray enters before t_max, sorted by entry distance.
  constexpr std::size_t kInline = 16;
  std::array<std::pair<float, std::uint32_t>, kInline> inline_buf;
  std::vector<std::pair<float, std::uint32_t>> heap_buf;
  std::pair<float, std::uint32_t> * order = inline_buf.data();
  if (surfaces.size() > kInline) {
    heap_buf.resize(surfaces.size());
    order = heap_buf.data();
  }
  std::size_t n = 0;
  for (std::size_t s = 0; s < surfaces.size(); ++s) {
    float t_in;
    if (!surfaces[s].bvh.empty() && ray_box_entry(surfaces[s].aabb, o, d, t_max, t_in)) {
      order[n++] = {t_in, static_cast<std::uint32_t>(s)};
    }
  }
  std::sort(order, order + n);

  bool any = false;
  float best_t = t_max;
  for (std::size_t k = 0; k < n && order[k].first < best_t; ++k) {
    NavCelId cid;
    float t;
    Vec3 p;
    if (surface_raycast(surfaces[order[k].second], o, d, cid, t, p, best_t) && t < best_t) {
      best_t = t;
      out = {true, order[k].second, cid, t, p};
      any = true;
    }
  }
  return any;
}

void NavMap::raycast_many(
  const std::vector<Ray> & rays,
  std::vector<RayHit> & out,
  bool first_hit_only) const
{
  out.clear();
  out.resize(rays.size());

  for (size_t i = 0; i < rays.size(); ++i) {
    const auto & ray = rays[i];
    if (!first_hit_only) {
      closest_hit_(ray.o, ray.d, std::numeric_limits<float>::infinity(), out[i]);
      continue;
    }
    for (size_t s = 0; s < surfaces.size(); ++s) {
      NavCelId cid;
      float t;
      Eigen::Vector3f p;
      if (surface_raycast(surfaces[s], ray.o, ray.d, cid, t, p)) {
        out[i] = {true, s, cid, t, p};
        break;
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Locate helpers
// -----------------------------------------------------------------------------

// Point of the plane (a, n) under/over p; along n if the plane is vertical.
static inline Vec3 project_to_plane_vertically(const Vec3 & p, const Vec3 & a, const Vec3 & n)
{
  const float dist = n.dot(p - a);
  if (std::fabs(n.z()) > 1e-6f) {
    return Vec3(p.x(), p.y(), p.z() - dist / n.z());
  }
  return p - dist * n;
}

static inline bool point_in_triangle_bary(
  const Vec3 & p,
  const Vec3 & a,
  const Vec3 & b,
  const Vec3 & c,
  Vec3 & bary,
  float eps)
{
  Vec3 v0 = b - a;
  Vec3 v1 = c - a;
  Vec3 v2 = p - a;
  float d00 = v0.dot(v0);
  float d01 = v0.dot(v1);
  float d11 = v1.dot(v1);
  float d20 = v2.dot(v0);
  float d21 = v2.dot(v1);
  float denom = d00 * d11 - d01 * d01;
  if (std::fabs(denom) < 1e-12f) {
    return false;
  }
  float v = (d11 * d20 - d01 * d21) / denom;
  float w = (d00 * d21 - d01 * d20) / denom;
  float u = 1.0f - v - w;
  bary = Vec3(u, v, w);
  return u >= -eps && v >= -eps && w >= -eps;
}

bool NavMap::locate_by_walking(
  NavCelId start_cid,
  const Vec3 & p,
  NavCelId & cid_out,
  Vec3 & bary,
  Vec3 * hit_pt,
  float planar_eps) const
{
  const int kMaxSteps = 16;
  NavCelId cid = start_cid;

  for (int step = 0; step < kMaxSteps; ++step) {
    const auto & c = navcels[cid];
    Vec3 a = positions.at(c.v[0]);
    Vec3 b = positions.at(c.v[1]);
    Vec3 d = positions.at(c.v[2]);
    Vec3 n = c.normal;
    Vec3 q = project_to_plane_vertically(p, a, n);
    if (hit_pt) {
      *hit_pt = q;
    }
    if (point_in_triangle_bary(q, a, b, d, bary, planar_eps)) {
      cid_out = cid;
      return true;
    }
    int neg_idx = -1;
    float min_val = 0.0f;
    for (int i = 0; i < 3; ++i) {
      if (bary[i] < min_val) {
        min_val = bary[i];
        neg_idx = i;
      }
    }
    if (neg_idx < 0) {
      cid_out = cid;
      return true;
    }
    // bary[i] < 0: p is beyond the edge opposite vertex i, i.e. edge (i+1)%3.
    NavCelId next = c.neighbor[(neg_idx + 1) % 3];
    if (next == std::numeric_limits<uint32_t>::max()) {
      cid_out = cid;
      return false;
    }
    cid = next;
  }
  cid_out = cid;
  return false;
}

// -----------------------------------------------------------------------------
// Locate API
// -----------------------------------------------------------------------------

bool NavMap::locate_navcel_core(
  const Vec3 & p_world,
  size_t & surface_idx,
  NavCelId & cid,
  Vec3 & bary,
  Vec3 * hit_pt,
  const LocateOpts & opts) const
{
  // 0) Fast path: directly test the hinted triangle, if any.
  if (opts.hint_cid.has_value()) {
    const NavCelId hint = *opts.hint_cid;
    if (hint < navcels.size()) {
      const auto & c = navcels[hint];
      const Vec3 a = positions.at(c.v[0]);
      const Vec3 b = positions.at(c.v[1]);
      const Vec3 d = positions.at(c.v[2]);
      const Vec3 & n = c.normal;

      const float dist = n.dot(p_world - a);
      const Vec3 q = project_to_plane_vertically(p_world, a, n);

      Vec3 bary_hint;
      if (point_in_triangle_bary(q, a, b, d, bary_hint, opts.planar_eps) &&
        std::fabs(dist) <= opts.height_eps)
      {
        cid = hint;
        bary = bary_hint;
        if (hit_pt) {
          *hit_pt = q;
        }

        const std::size_t s = surface_of(cid);
        if (s != kNoSurface) {
          surface_idx = s;
          return true;
        }
        // If no surface owns the hinted navcel, fall through to the generic search.
      }
    }
  }

  // 1) Try walking if there is a valid hint and we are not far from its plane.
  if (opts.hint_cid.has_value()) {
    const NavCelId start = *opts.hint_cid;
    if (start < navcels.size()) {
      const auto & c0 = navcels[start];
      const Vec3 a0 = positions.at(c0.v[0]);
      const Vec3 & n0 = c0.normal;
      const float dist0 = n0.dot(p_world - a0);

      // Do not walk if the query point is clearly off the hinted plane.
      Vec3 q;
      if (std::fabs(dist0) <= opts.height_eps) {
        // The walk may end on a cell far from p in height: re-check height_eps there.
        if (locate_by_walking(start, p_world, cid, bary, &q, opts.planar_eps) &&
          (p_world - q).norm() <= opts.height_eps)
        {
          if (hit_pt) {*hit_pt = q;}
          const std::size_t s = surface_of(cid);
          if (s != kNoSurface) {
            surface_idx = s;
            return true;
          }
          // Fall through if surface not found.
        }
      }
    }
  }

  // 3) Vertical rays down and up; keep the surface closest in z.
  {
    // Start slightly past p so a surface through p itself is hit (t > 0).
    constexpr float kRayLift = 1e-3f;
    const float inf = std::numeric_limits<float>::infinity();

    auto valid_hit = [&](const RayHit & h, Vec3 & bary_out) {
        const auto & tri = navcels[h.cid];
        return point_in_triangle_bary(
          h.p, positions.at(tri.v[0]), positions.at(tri.v[1]), positions.at(tri.v[2]),
          bary_out, opts.planar_eps);
      };

    RayHit down, up;
    Vec3 bary_down, bary_up;
    const bool has_down =
      closest_hit_(p_world + Vec3(0.0f, 0.0f, kRayLift), Vec3(0.0f, 0.0f, -1.0f), inf, down) &&
      valid_hit(down, bary_down);
    const float dz_down = has_down ? std::fabs(down.p.z() - p_world.z()) : inf;

    // Upward ray only needs surfaces strictly closer than the one below.
    const float up_max = has_down ? dz_down + kRayLift : inf;
    const bool has_up =
      closest_hit_(p_world - Vec3(0.0f, 0.0f, kRayLift), Vec3(0.0f, 0.0f, 1.0f), up_max, up) &&
      valid_hit(up, bary_up) && std::fabs(up.p.z() - p_world.z()) < dz_down;

    if (has_down || has_up) {
      const RayHit & best = has_up ? up : down;
      surface_idx = best.surface;
      cid = best.cid;
      bary = has_up ? bary_up : bary_down;
      if (hit_pt) {*hit_pt = best.p;}
      return true;
    }
  }
  return false;
}

// -----------------------------------------------------------------------------
// Closest triangle queries
// -----------------------------------------------------------------------------

bool NavMap::surface_closest_navcel(
  const Surface & s,
  const Vec3 & p,
  NavCelId & cid,
  Vec3 & q,
  float & best_sq) const
{
  if (s.bvh.empty()) {
    return false;
  }

  bool any = false;
  NodeStack st;
  st.push(0);

  while (!st.empty()) {
    const BVHNode & n = s.bvh[st.pop()];
    if (box_sq_dist(n.box, p) >= best_sq) {
      continue;
    }

    if (n.is_leaf()) {
      for (int i = 0; i < n.count; ++i) {
        int cid_local = s.prim_indices[n.start + i];
        const auto & c = navcels[cid_local];
        Vec3 a = positions.at(c.v[0]);
        Vec3 b = positions.at(c.v[1]);
        Vec3 d = positions.at(c.v[2]);
        Vec3 qp;
        float sd = point_triangle_squared_distance(p, a, b, d, &qp);
        if (sd < best_sq) {
          best_sq = sd;
          q = qp;
          cid = cid_local;
          any = true;
        }
      }
    } else if (n.left >= 0 && n.right >= 0) {
      // Push the farther child first so the nearer one is visited first.
      const bool left_near =
        box_sq_dist(s.bvh[n.left].box, p) <= box_sq_dist(s.bvh[n.right].box, p);
      st.push(left_near ? n.right : n.left);
      st.push(left_near ? n.left : n.right);
    } else {
      if (n.left >= 0) {st.push(n.left);}
      if (n.right >= 0) {st.push(n.right);}
    }
  }
  return any;
}

bool NavMap::closest_navcel(
  const Vec3 & p_world,
  size_t & surface_idx,
  NavCelId & cid,
  Vec3 & closest_point,
  float & sqdist,
  int restrict_surface) const
{
  bool any = false;
  float best_sq = std::numeric_limits<float>::infinity();
  Vec3 best_q(0.0f, 0.0f, 0.0f);
  NavCelId best_cid = 0;
  size_t best_s = 0;

  size_t s_begin = 0;
  size_t s_end = surfaces.size();
  if (restrict_surface >= 0) {
    s_begin = static_cast<size_t>(restrict_surface);
    s_end = static_cast<size_t>(restrict_surface + 1);
  }

  for (size_t s = s_begin; s < s_end; ++s) {
    const auto & surf = surfaces[s];

    float dx = std::max(
      std::max(surf.aabb.min.x() - p_world.x(), 0.0f),
      p_world.x() - surf.aabb.max.x());
    float dy = std::max(
      std::max(surf.aabb.min.y() - p_world.y(), 0.0f),
      p_world.y() - surf.aabb.max.y());
    float dz = std::max(
      std::max(surf.aabb.min.z() - p_world.z(), 0.0f),
      p_world.z() - surf.aabb.max.z());
    float aabb_lb = dx * dx + dy * dy + dz * dz;
    if (aabb_lb > best_sq) {
      continue;
    }

    NavCelId cid_s = 0;
    Vec3 q_s(0.0f, 0.0f, 0.0f);
    float best_sq_s = best_sq;
    if (surface_closest_navcel(surf, p_world, cid_s, q_s, best_sq_s)) {
      if (best_sq_s < best_sq) {
        best_sq = best_sq_s;
        best_q = q_s;
        best_cid = cid_s;
        best_s = s;
        any = true;
      }
    }
  }

  if (!any) {
    return false;
  }
  surface_idx = best_s;
  cid = best_cid;
  closest_point = best_q;
  sqdist = best_sq;
  return true;
}

std::size_t NavMap::create_surface(std::string frame_id)
{
  surfaces.push_back(Surface{});
  surfaces.back().frame_id = std::move(frame_id);
  return surfaces.size() - 1;
}

Surface NavMap::create_surface_obj(const std::string & frame_id) const
{
  Surface s;
  s.frame_id = frame_id;
  return s;
}

std::size_t NavMap::add_surface(const Surface & s)
{
  surfaces.push_back(s);
  return surfaces.size() - 1;
}

std::size_t NavMap::add_surface(Surface && s)
{
  surfaces.push_back(std::move(s));
  return surfaces.size() - 1;
}

bool NavMap::remove_surface(std::size_t surface_index)
{
  if (surface_index >= surfaces.size()) {return false;}
  surfaces.erase(surfaces.begin() + surface_index);
  return true;
}

uint32_t NavMap::add_vertex(const Eigen::Vector3f & p)
{
  positions.x.push_back(p.x());
  positions.y.push_back(p.y());
  positions.z.push_back(p.z());
  return static_cast<uint32_t>(positions.x.size() - 1);
}

NavCelId NavMap::add_navcel(uint32_t v0, uint32_t v1, uint32_t v2)
{
  NavCel t; t.v[0] = v0; t.v[1] = v1; t.v[2] = v2;
  navcels.push_back(t);
  return static_cast<NavCelId>(navcels.size() - 1);
}

void NavMap::add_navcel_to_surface(std::size_t surface_index, NavCelId cid)
{
  if (surface_index >= surfaces.size()) {return;}
  surfaces[surface_index].navcels.push_back(cid);
}

// ---- Geometry helpers ----
Eigen::Vector3f NavMap::navcel_centroid(NavCelId cid) const
{
  const auto & t = navcels[cid];
  Eigen::Vector3f a(positions.x.at(t.v[0]), positions.y.at(t.v[0]), positions.z.at(t.v[0]));
  Eigen::Vector3f b(positions.x.at(t.v[1]), positions.y.at(t.v[1]), positions.z.at(t.v[1]));
  Eigen::Vector3f c(positions.x.at(t.v[2]), positions.y.at(t.v[2]), positions.z.at(t.v[2]));
  return (a + b + c) / 3.f;
}

std::vector<NavCelId> NavMap::navcel_neighbors(NavCelId cid) const
{
  std::vector<NavCelId> out; out.reserve(3);
  const auto & t = navcels[cid];
  for (int e = 0; e < 3; ++e) {
    auto n = t.neighbor[e];
    if (n != std::numeric_limits<uint32_t>::max()) {out.push_back(n);}
  }
  return out;
}

void NavMap::build_navcel_surface_index_()
{
  constexpr auto kUnset = std::numeric_limits<std::uint32_t>::max();
  navcel_surface_.assign(navcels.size(), kUnset);
  for (std::size_t s = 0; s < surfaces.size(); ++s) {
    for (auto cid : surfaces[s].navcels) {
      // First owner wins, as in the linear fallback.
      if (cid < navcel_surface_.size() && navcel_surface_[cid] == kUnset) {
        navcel_surface_[cid] = static_cast<std::uint32_t>(s);
      }
    }
  }
}

std::size_t NavMap::surface_of(NavCelId cid) const
{
  if (cid >= navcels.size()) {return kNoSurface;}

  // Stale index (geometry edited without a rebuild): linear fallback.
  if (navcel_surface_.size() != navcels.size()) {
    for (std::size_t s = 0; s < surfaces.size(); ++s) {
      const auto & ids = surfaces[s].navcels;
      if (std::find(ids.begin(), ids.end(), cid) != ids.end()) {return s;}
    }
    return kNoSurface;
  }

  const auto s = navcel_surface_[cid];
  return (s == std::numeric_limits<std::uint32_t>::max() || s >= surfaces.size()) ?
         kNoSurface : static_cast<std::size_t>(s);
}

bool NavMap::locate_navcel(
  const Eigen::Vector3f & p_world,
  std::size_t & surface_idx,
  NavCelId & cid,
  Eigen::Vector3f & bary,
  Eigen::Vector3f * hit_pt,
  const LocateOpts & opts) const
{
  if (locate_navcel_core(p_world, surface_idx, cid, bary, hit_pt, opts)) {
    return true;
  }

  const float R = 1e6f;
  const Eigen::Vector3f up_origin = {p_world.x(), p_world.y(), p_world.z() + R};
  const Eigen::Vector3f down_origin = {p_world.x(), p_world.y(), p_world.z() - R};
  const Eigen::Vector3f up_dir = {0.0f, 0.0f, -1.0f};
  const Eigen::Vector3f down_dir = {0.0f, 0.0f, 1.0f};

  struct Candidate
  {
    bool ok{false};
    std::size_t surf{};
    NavCelId cid{};
    Eigen::Vector3f hit{};
    Eigen::Vector3f bary{};
    float dz{std::numeric_limits<float>::infinity()};
  };

  auto make_candidate = [&](const Eigen::Vector3f & o, const Eigen::Vector3f & d) -> Candidate {
      Candidate c;
      NavCelId hit_cid{};
      float t{};
      Eigen::Vector3f hit{};

      if (raycast(o, d, hit_cid, t, hit)) {
        c.ok = true;
        c.cid = hit_cid;
        c.hit = hit;
        c.dz = std::fabs(hit.z() - p_world.z());

        const std::size_t si = surface_of(hit_cid);
        c.surf = (si != kNoSurface) ? si : 0;

        const auto & tri = navcels[c.cid];
        const Eigen::Vector3f A = positions.at(tri.v[0]);
        const Eigen::Vector3f B = positions.at(tri.v[1]);
        const Eigen::Vector3f C = positions.at(tri.v[2]);

        const Eigen::Vector3f v0 = B - A;
        const Eigen::Vector3f v1 = C - A;
        const Eigen::Vector3f v2 = hit - A;
        const float d00 = v0.dot(v0);
        const float d01 = v0.dot(v1);
        const float d11 = v1.dot(v1);
        const float d20 = v2.dot(v0);
        const float d21 = v2.dot(v1);
        const float denom = d00 * d11 - d01 * d01;
        if (std::fabs(denom) > 1e-20f) {
          const float v = (d11 * d20 - d01 * d21) / denom;
          const float w = (d00 * d21 - d01 * d20) / denom;
          const float u = 1.0f - v - w;
          c.bary = Eigen::Vector3f(u, v, w);
        } else {
          c.bary = Eigen::Vector3f(1, 0, 0);
        }
      }
      return c;
    };

  Candidate cand_up = make_candidate(up_origin, up_dir);
  Candidate cand_down = make_candidate(down_origin, down_dir);

  if (cand_up.ok && cand_up.dz > opts.height_eps) {cand_up.ok = false;}
  if (cand_down.ok && cand_down.dz > opts.height_eps) {cand_down.ok = false;}

  const Candidate * best = nullptr;
  if (cand_up.ok && cand_down.ok) {
    best = (cand_up.dz <= cand_down.dz) ? &cand_up : &cand_down;
  } else if (cand_up.ok) {
    best = &cand_up;
  } else if (cand_down.ok) {
    best = &cand_down;
  }

  if (!best) {
    return false;
  }

  surface_idx = best->surf;
  cid = best->cid;
  bary = best->bary;
  if (hit_pt) {*hit_pt = best->hit;}
  return true;
}

// ---- Distance field ----

namespace
{
inline float point_segment_distance(const Vec3 & p, const Vec3 & a, const Vec3 & b)
{
  const Vec3 ab = b - a;
  const float len2 = ab.squaredNorm();
  const float t = len2 > 0.0f ? std::clamp((p - a).dot(ab) / len2, 0.0f, 1.0f) : 0.0f;
  return (p - (a + t * ab)).norm();
}
}  // namespace

std::vector<float> NavMap::distance_field(
  const std::vector<NavCelId> & seed_cells, bool from_boundary, float max_distance) const
{
  constexpr float kInf = std::numeric_limits<float>::infinity();
  constexpr auto kNone = std::numeric_limits<NavCelId>::max();
  const std::size_t n = navcels.size();
  std::vector<float> dist(n, kInf);
  if (n == 0) {return dist;}

  std::vector<Vec3> centroid(n);
  for (NavCelId c = 0; c < n; ++c) {
    centroid[c] = navcel_centroid(c);
  }

  // A seed is a point (a == b) or a segment.
  struct Seed {Vec3 a, b;};
  std::vector<Seed> seeds;
  std::vector<std::uint32_t> nearest(n, std::numeric_limits<std::uint32_t>::max());
  using Item = std::pair<float, NavCelId>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;

  auto offer = [&](NavCelId c, std::uint32_t seed, float d) {
      if (d < dist[c] && d <= max_distance) {
        dist[c] = d;
        nearest[c] = seed;
        open.push({d, c});
      }
    };
  auto distance_to = [&](NavCelId c, std::uint32_t seed) {
      return point_segment_distance(centroid[c], seeds[seed].a, seeds[seed].b);
    };

  for (NavCelId c : seed_cells) {
    if (c >= n) {continue;}
    seeds.push_back({centroid[c], centroid[c]});
    offer(c, static_cast<std::uint32_t>(seeds.size() - 1), 0.0f);
  }
  if (from_boundary) {
    for (NavCelId c = 0; c < n; ++c) {
      for (int e = 0; e < 3; ++e) {
        if (navcels[c].neighbor[e] != kNone) {continue;}
        seeds.push_back({positions.at(navcels[c].v[e]), positions.at(navcels[c].v[(e + 1) % 3])});
        const auto seed = static_cast<std::uint32_t>(seeds.size() - 1);
        offer(c, seed, distance_to(c, seed));
      }
    }
  }

  while (!open.empty()) {
    const auto [d, u] = open.top();
    open.pop();
    if (d > dist[u]) {continue;}
    for (int e = 0; e < 3; ++e) {
      const NavCelId v = navcels[u].neighbor[e];
      if (v == kNone || v >= n) {continue;}
      offer(v, nearest[u], distance_to(v, nearest[u]));
    }
  }
  return dist;
}

// ---- Layers ----
bool NavMap::has_layer(const std::string & name) const
{
  return static_cast<bool>(layers.get(name));
}

std::size_t NavMap::layer_size(const std::string & name) const
{
  if (auto v = layers.get(name)) {return v->size();}
  return 0;
}

std::string NavMap::layer_type_name(const std::string & name) const
{
  if (auto v = layers.get(name)) {
    switch (v->type()) {
      case LayerType::U8:  return "uint8";
      case LayerType::F32: return "float";
      case LayerType::F64: return "double";
      default: break;
    }
  }
  return "unknown";
}

std::optional<LayerMeta> NavMap::get_layer_meta(const std::string & name) const
{
  auto it = layer_meta.find(name);
  if (it == layer_meta.end()) {return std::nullopt;}
  return it->second;
}

std::vector<std::string> NavMap::list_layers() const
{
  return layers.list();
}

double NavMap::sample_layer_at(
  const std::string & name,
  const Eigen::Vector3f & p_world,
  double def) const
{
  // Si no existe la capa, devuelve directamente el valor por defecto
  if (!has_layer(name)) {
    return def;
  }

  std::size_t s; NavCelId c; Eigen::Vector3f bary; Eigen::Vector3f hit;
  if (!locate_navcel(p_world, s, c, bary, &hit)) {
    return def;
  }

  double v = layer_get<double>(name, c, std::numeric_limits<double>::quiet_NaN());
  if (std::isnan(v)) {
    return def;
  }
  return v;
}


}  // namespace navmap
