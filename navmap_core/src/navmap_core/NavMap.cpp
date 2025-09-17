// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.


#include "navmap_core/NavMap.hpp"
#include <algorithm>
#include <functional>

namespace navmap
{

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
  float planar_eps)
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

bool NavMap::locate_navcel(
  const Vec3 & p_world,
  size_t & surface_idx,
  NavCelId & cid,
  Vec3 & bary,
  Vec3 * hit_pt,
  const LocateOpts & opts)
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

  // 3) Vertical raycast fallback (downward or upward).
  Vec3 dir = opts.use_downward_ray ? Vec3(0.0f, 0.0f, -1.0f) : Vec3(0.0f, 0.0f, 1.0f);
  float best_dz = std::numeric_limits<float>::infinity();
  bool any = false;
  Vec3 best_hit(0.0f, 0.0f, 0.0f);

  for (size_t s = 0; s < surfaces.size(); ++s) {
    const auto & surf = surfaces[s];
    if (!surf.aabb.contains_xy_only(p_world)) {
      continue;
    }
    NavCelId cid_s;
    float t;
    Vec3 hit;
    if (surface_raycast(surf, p_world, dir, cid_s, t, hit)) {
      float dz = std::fabs(hit.z() - p_world.z());
      if (dz < best_dz) {
        const auto & c = navcels[cid_s];
        Vec3 a = positions.at(c.v[0]);
        Vec3 b = positions.at(c.v[1]);
        Vec3 d = positions.at(c.v[2]);
        Vec3 bary_tmp;
        if (point_in_triangle_bary(hit, a, b, d, bary_tmp, opts.planar_eps)) {
          best_dz = dz;
          best_hit = hit;
          cid = cid_s;
          bary = bary_tmp;
          surface_idx = s;
          any = true;
        }
      }
    }
  }

  if (any) {
    if (hit_pt) {
      *hit_pt = best_hit;
    }
    return true;
  }

  // 4) Optional upward retry if downward failed.
  if (opts.use_downward_ray) {
    LocateOpts opt2 = opts;
    opt2.use_downward_ray = false;
    return locate_navcel(p_world, surface_idx, cid, bary, hit_pt, opt2);
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

}  // namespace navmap
