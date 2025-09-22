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
#include <algorithm>
#include <functional>
#include <cmath>
#include <limits>
#include <cstdint>
#include <unordered_set>

namespace navmap
{

/** @cond INTERNAL */
namespace navmap
{namespace detail
{
inline std::uint64_t fnv1a64_bytes(
  const void * data, std::size_t n,
  std::uint64_t seed = 1469598103934665603ULL)
{
  const auto * p = static_cast<const std::uint8_t *>(data);
  std::uint64_t h = seed;
  for (std::size_t i = 0; i < n; ++i) {
    h ^= p[i]; h *= 1099511628211ULL;
  }
  return h;
}
}}  // namespaces
/** @endcond */

template<typename T>
std::uint64_t LayerView<T>::content_hash() const
{
  if (!hash_dirty_) {return hash_cache_;}
  const std::size_t n = data_.size();
  std::uint64_t h = navmap::detail::fnv1a64_bytes(&n, sizeof(n));
  if (n) {
    static_assert(std::is_trivially_copyable<T>::value,
        "LayerView<T> requires trivially copyable T.");
    h = navmap::detail::fnv1a64_bytes(data_.data(), n * sizeof(T), h);
  }
  hash_cache_ = h;
  hash_dirty_ = false;
  return hash_cache_;
}

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
} // namespace

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
  const float *x, std::size_t nx,
  const float *y, std::size_t ny,
  const float *z, std::size_t nz,
  const std::uint32_t *v0, std::size_t nv0,
  const std::uint32_t *v1, std::size_t nv1,
  const std::uint32_t *v2, std::size_t nv2)
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
  struct EdgeKey { PointId a; PointId b; };
  struct EdgeHash
  {
    size_t operator()(const EdgeKey & k) const
    {
      return static_cast<size_t>(k.a) * 73856093u ^
             static_cast<size_t>(k.b) * 19349663u;
    }
  };
  struct EdgeEq
  {
    bool operator()(const EdgeKey & x, const EdgeKey & y) const
    {
      return x.a == y.a && x.b == y.b;
    }
  };

  std::unordered_map<EdgeKey, std::pair<NavCelId, int>, EdgeHash, EdgeEq> E;
  for (NavCelId cid = 0; cid < navcels.size(); ++cid) {
    auto & c = navcels[cid];
    for (int e = 0; e < 3; ++e) {
      PointId a = c.v[e];
      PointId b = c.v[(e + 1) % 3];
      if (a > b) {std::swap(a, b);}
      EdgeKey key{a, b};
      auto it = E.find(key);
      if (it == E.end()) {
        E.emplace(key, std::make_pair(cid, e));
      } else {
        auto pair = it->second;
        auto cid2 = pair.first;
        auto e2 = pair.second;
        navcels[cid].neighbor[e] = cid2;
        navcels[cid2].neighbor[e2] = cid;
      }
    }
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
      std::nth_element(prims.begin() + begin,
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
  Vec3 & hit_pt) const
{
  if (s.bvh.empty()) {
    return false;
  }

  bool hit = false;
  float best_t = std::numeric_limits<float>::infinity();
  std::stack<int> st;
  st.push(0);

  while (!st.empty()) {
    int idx = st.top();
    st.pop();
    const BVHNode & n = s.bvh[idx];
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
  bool any = false;
  float best_t = std::numeric_limits<float>::infinity();
  Vec3 best_p;
  NavCelId best_cid = 0;

  for (const auto & s : surfaces) {
    NavCelId cid;
    float tt;
    Vec3 p;
    if (surface_raycast(s, o, d, cid, tt, p)) {
      if (tt < best_t) {
        best_t = tt;
        best_p = p;
        best_cid = cid;
        any = true;
      }
    }
  }

  if (any) {
    hit_cid = best_cid;
    t = best_t;
    hit_pt = best_p;
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
    bool any = false;
    float best_t = std::numeric_limits<float>::infinity();
    Eigen::Vector3f best_p(0, 0, 0);
    NavCelId best_cid = 0;
    size_t best_surf = 0;

    for (size_t s = 0; s < surfaces.size(); ++s) {
      NavCelId cid;
      float t;
      Eigen::Vector3f p;
      if (surface_raycast(surfaces[s], ray.o, ray.d, cid, t, p)) {
        if (first_hit_only) {
          out[i] = {true, s, cid, t, p};
          any = true;
          break;
        } else {
          if (t < best_t) {
            best_t = t;
            best_p = p;
            best_cid = cid;
            best_surf = s;
            any = true;
          }
        }
      }
    }

    if (!first_hit_only && any) {
      out[i] = {true, best_surf, best_cid, best_t, best_p};
    }
  }
}

// -----------------------------------------------------------------------------
// Locate helpers
// -----------------------------------------------------------------------------

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
  return  u >= -eps && v >= -eps && w >= -eps;
}

bool NavMap::locate_by_walking(
  NavCelId start_cid,
  const Vec3 & p,
  NavCelId & cid_out,
  Vec3 & bary,
  Vec3 * hit_pt,
  float planar_eps) const
{
  const int kMaxSteps = 64;
  NavCelId cid = start_cid;

  for (int step = 0; step < kMaxSteps; ++step) {
    const auto & c = navcels[cid];
    Vec3 a = positions.at(c.v[0]);
    Vec3 b = positions.at(c.v[1]);
    Vec3 d = positions.at(c.v[2]);
    Vec3 n = c.normal;
    float dist = n.dot(p - a);
    Vec3 q = p - dist * n;
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
    NavCelId next = c.neighbor[neg_idx];
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
  // 1) Try walking if there is a valid hint.
  if (opts.hint_cid.has_value()) {
    if (locate_by_walking(opts.hint_cid.value(),
                          p_world,
                          cid,
                          bary,
                          hit_pt,
                          opts.planar_eps))
    {
      for (size_t s = 0; s < surfaces.size(); ++s) {
        const auto & surf = surfaces[s];
        if (std::find(surf.navcels.begin(), surf.navcels.end(), cid) !=
          surf.navcels.end())
        {
          surface_idx = s;
          return true;
        }
      }
      // Fall through if surface not found.
    }
  }

  // 3) Vertical raycast fallback (choose by |dz| using BOTH directions)
  {
    // Shoot both rays: downward and upward
    std::vector<Ray> rays;
    rays.push_back({p_world, Vec3(0.0f, 0.0f, -1.0f)});
    rays.push_back({p_world, Vec3(0.0f, 0.0f, 1.0f)});

    std::vector<RayHit> hits;
    raycast_many(rays, hits, /*first_hit_only=*/true);

    bool any = false;
    float best_dz = std::numeric_limits<float>::infinity();
    size_t best_surf = 0;
    NavCelId best_cid = 0;
    Vec3 best_hit(0.0f, 0.0f, 0.0f);
    Vec3 best_bary(0.0f, 0.0f, 0.0f);

    for (const auto & h : hits) {
      if (!h.hit) {continue;}
      const float dz = std::fabs(h.p.z() - p_world.z());
      // Compute barycentrics on the hit triangle to return consistent outputs
      const auto & tri = navcels[h.cid];
      const Vec3 a = positions.at(tri.v[0]);
      const Vec3 b = positions.at(tri.v[1]);
      const Vec3 c = positions.at(tri.v[2]);
      Vec3 bary_tmp;
      if (!point_in_triangle_bary(h.p, a, b, c, bary_tmp, opts.planar_eps)) {
        continue; // numerical guard
      }
      if (dz < best_dz) {
        best_dz = dz;
        best_surf = h.surface;
        best_cid = h.cid;
        best_hit = h.p;
        best_bary = bary_tmp;
        any = true;
      }
    }

    if (any) {
      surface_idx = best_surf;
      cid = best_cid;
      bary = best_bary;
      if (hit_pt) {*hit_pt = best_hit;}
      return true;
    }
  }

  return false;
}

// -----------------------------------------------------------------------------
// Closest triangle queries
// -----------------------------------------------------------------------------

bool NavMap::surface_closest_triangle(
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
  std::stack<int> st;
  st.push(0);

  while (!st.empty()) {
    int idx = st.top();
    st.pop();
    const BVHNode & n = s.bvh[idx];

    float dx = std::max(std::max(n.box.min.x() - p.x(), 0.0f), p.x() - n.box.max.x());
    float dy = std::max(std::max(n.box.min.y() - p.y(), 0.0f), p.y() - n.box.max.y());
    float dz = std::max(std::max(n.box.min.z() - p.z(), 0.0f), p.z() - n.box.max.z());
    float box_lb = dx * dx + dy * dy + dz * dz;
    if (box_lb > best_sq) {
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
    } else {
      if (n.left >= 0) {st.push(n.left);}
      if (n.right >= 0) {st.push(n.right);}
    }
  }
  return any;
}

bool NavMap::closest_triangle(
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

    float dx = std::max(std::max(surf.aabb.min.x() - p_world.x(), 0.0f),
                        p_world.x() - surf.aabb.max.x());
    float dy = std::max(std::max(surf.aabb.min.y() - p_world.y(), 0.0f),
                        p_world.y() - surf.aabb.max.y());
    float dz = std::max(std::max(surf.aabb.min.z() - p_world.z(), 0.0f),
                        p_world.z() - surf.aabb.max.z());
    float aabb_lb = dx * dx + dy * dy + dz * dz;
    if (aabb_lb > best_sq) {
      continue;
    }

    NavCelId cid_s = 0;
    Vec3 q_s(0.0f, 0.0f, 0.0f);
    float best_sq_s = best_sq;
    if (surface_closest_triangle(surf, p_world, cid_s, q_s, best_sq_s)) {
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

  const float R = 1e6f; // altura grande para garantizar intersección si existe
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

  auto make_candidate = [&](const Eigen::Vector3f & o,
    const Eigen::Vector3f & d) -> Candidate {
      Candidate c;
      NavCelId hit_cid{};
      float t{};
      Eigen::Vector3f hit{};

      if (raycast(o, d, hit_cid, t, hit)) {
        c.ok = true;
        c.cid = hit_cid;
        c.hit = hit;
        c.dz = std::fabs(hit.z() - p_world.z());

        for (std::size_t si = 0; si < surfaces.size(); ++si) {
          const auto & s = surfaces[si];
          if (std::find(s.navcels.begin(), s.navcels.end(), hit_cid) != s.navcels.end()) {
            c.surf = si;
            break;
          }
        }

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
