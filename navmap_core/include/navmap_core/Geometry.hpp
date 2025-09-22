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

/**
 * @file Geometry.hpp
 * @brief Low-level geometry utilities for NavMap cells (triangles), AABB and
 *        basic queries such as ray-triangle intersection and closest-point.
 *
 * This header is intentionally lightweight and header-only to enable
 * aggressive inlining by the compiler. It provides:
 *  - Basic triangle primitives (area, normal).
 *  - Robust Möller–Trumbore ray–triangle intersection.
 *  - AABB with robust ray-box intersection and XY containment helpers.
 *  - Closest point on triangle and point–triangle distance.
 */

#ifndef NAVMAP_CORE__GEOMETRY_HPP
#define NAVMAP_CORE__GEOMETRY_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <cmath>
#include <algorithm>
#include <array>

namespace navmap
{

/**
 * @brief 3D vector alias used across NavMap geometry.
 */
using Vec3 = Eigen::Vector3f;

// -----------------------------------------------------------------------------
// Basic triangle geometry
// -----------------------------------------------------------------------------

/**
 * @brief Compute the area of a triangle ABC.
 *
 * @param a First vertex (world coordinates).
 * @param b Second vertex (world coordinates).
 * @param c Third vertex (world coordinates).
 * @return Triangle area (non-negative).
 *
 * @note Uses 0.5 * |(b - a) x (c - a)|.
 */
inline float triangle_area(const Vec3 & a, const Vec3 & b, const Vec3 & c)
{
  return 0.5f * ((b - a).cross(c - a)).norm();
}

/**
 * @brief Compute a unit normal of triangle ABC.
 *
 * The normal is computed as normalized cross product (b - a) x (c - a).
 * If the triangle is degenerate (near-zero area), returns +Z (0, 0, 1).
 *
 * @param a First vertex (world coordinates).
 * @param b Second vertex (world coordinates).
 * @param c Third vertex (world coordinates).
 * @return Unit-length normal or +Z fallback for degenerate triangles.
 */
inline Vec3 triangle_normal(const Vec3 & a, const Vec3 & b, const Vec3 & c)
{
  Vec3 n = (b - a).cross(c - a);
  float len = n.norm();
  return len > 1e-12f ? n / len : Vec3(0.0f, 0.0f, 1.0f);
}

/**
 * @brief Möller–Trumbore ray–triangle intersection.
 *
 * Solves intersection between a ray (orig + t * dir, t > 0) and triangle
 * (v0, v1, v2). Returns @c true on hit and outputs:
 *  - @p t: parametric distance along the ray (t > 0).
 *  - @p u, @p v: barycentric coordinates (with w = 1 - u - v).
 *
 * @param orig Ray origin.
 * @param dir Ray direction (not necessarily unit length).
 * @param v0 First triangle vertex.
 * @param v1 Second triangle vertex.
 * @param v2 Third triangle vertex.
 * @param t Output ray parameter on hit (> 0).
 * @param u Output barycentric u on hit.
 * @param v Output barycentric v on hit.
 * @return True if the ray intersects the triangle with t > 0.
 *
 * @warning Numerical robustness depends on the geometry scale; @c kEps is
 *          set conservatively for typical mapping units (meters).
 */
inline bool ray_triangle_intersect(
  const Vec3 & orig,
  const Vec3 & dir,
  const Vec3 & v0,
  const Vec3 & v1,
  const Vec3 & v2,
  float & t,
  float & u,
  float & v)
{
  const float kEps = 1e-8f;
  Vec3 e1 = v1 - v0;
  Vec3 e2 = v2 - v0;
  Vec3 pvec = dir.cross(e2);
  float det = e1.dot(pvec);
  if (std::fabs(det) < kEps) {
    return false;
  }
  float inv_det = 1.0f / det;
  Vec3 tvec = orig - v0;
  u = tvec.dot(pvec) * inv_det;
  if (u < -kEps || u > 1.0f + kEps) {
    return false;
  }
  Vec3 qvec = tvec.cross(e1);
  v = dir.dot(qvec) * inv_det;
  if (v < -kEps || (u + v) > 1.0f + kEps) {
    return false;
  }
  t = e2.dot(qvec) * inv_det;
  return t > kEps;
}

// -----------------------------------------------------------------------------
// Axis-aligned bounding box (AABB)
// -----------------------------------------------------------------------------

/**
 * @brief Axis-aligned bounding box.
 *
 * Stores component-wise minima/maxima and provides helpers for:
 *  - Union/expansion with points or other AABBs.
 *  - Longest axis query (for BVH splitting).
 *  - Robust ray-box intersection (slabs method).
 *  - XY containment with and without Z tolerance.
 */
struct AABB
{
  /**
   * @brief Minimum corner (x_min, y_min, z_min). Initialized to +inf.
   */
  Vec3 min{std::numeric_limits<float>::infinity(),
    std::numeric_limits<float>::infinity(),
    std::numeric_limits<float>::infinity()};
  /**
   * @brief Maximum corner (x_max, y_max, z_max). Initialized to -inf.
   */
  Vec3 max{-std::numeric_limits<float>::infinity(),
    -std::numeric_limits<float>::infinity(),
    -std::numeric_limits<float>::infinity()};

  /**
   * @brief Expand the box to include a point @p p.
   * @param p Point to merge into this AABB.
   */
  inline void expand(const Vec3 & p)
  {
    min = min.cwiseMin(p);
    max = max.cwiseMax(p);
  }

  /**
   * @brief Expand the box to include another box @p b (component-wise union).
   * @param b Other AABB to merge.
   */
  inline void expand(const AABB & b)
  {
    min = min.cwiseMin(b.min);
    max = max.cwiseMax(b.max);
  }

  /**
   * @brief Return the index of the longest axis (0=x, 1=y, 2=z).
   * @return Axis index with the largest extent.
   */
  inline int longest_axis() const
  {
    Vec3 d = max - min;
    if (d.x() >= d.y() && d.x() >= d.z()) {return 0;}
    if (d.y() >= d.x() && d.y() >= d.z()) {return 1;}
    return 2;
  }

  /**
   * @brief Robust ray-box intersection (slabs method).
   *
   * Intersects ray (o + t * d) against the box. Handles zero components in
   * direction @p d by checking slab containment on that axis.
   *
   * @param o Ray origin.
   * @param d Ray direction.
   * @param tmax Upper bound for @p t interval (can be +inf).
   * @return True if the ray intersects the box with some t in [0, tmax].
   */
  inline bool intersects_ray(
    const Vec3 & o,
    const Vec3 & d,
    float tmax = 1e30f) const
  {
    const float kEps = 1e-12f;
    float tmin_local = 0.0f;

    for (int i = 0; i < 3; ++i) {
      const float di = d[i];
      const float oi = o[i];
      const float min_i = min[i];
      const float max_i = max[i];

      if (std::fabs(di) < kEps) {
        // Parallel to this slab: must be inside [min_i, max_i]
        if (oi < min_i || oi > max_i) {return false;}
        continue;
      }

      float inv = 1.0f / di;
      float t0 = (min_i - oi) * inv;
      float t1 = (max_i - oi) * inv;
      if (t0 > t1) {std::swap(t0, t1);}

      if (t0 > tmin_local) {tmin_local = t0;}
      if (t1 < tmax) {tmax = t1;}

      if (tmax < tmin_local) {return false;}
    }
    return tmax >= tmin_local && tmax > 0.0f;
  }

  /**
   * @brief 2D containment test on XY with tolerance in Z.
   *
   * Checks if point @p p lies inside the XY rectangle and within a vertical
   * band around [min.z, max.z] expanded by @p z_eps.
   *
   * @param p Query point.
   * @param z_eps Extra vertical half-thickness tolerance.
   * @return True if inside XY and |z - [min.z,max.z]| <= z_eps.
   */
  inline bool contains_xy(const Vec3 & p, float z_eps) const
  {
    return  p.x() >= min.x() && p.x() <= max.x() &&
           p.y() >= min.y() && p.y() <= max.y() &&
           p.z() >= min.z() - z_eps && p.z() <= max.z() + z_eps;
  }

  /**
   * @brief 2D containment test on XY only (no Z check).
   *
   * @param p Query point.
   * @return True if inside XY rectangle [min,max].
   */
  inline bool contains_xy_only(const Vec3 & p) const
  {
    return  p.x() >= min.x() && p.x() <= max.x() &&
           p.y() >= min.y() && p.y() <= max.y();
  }
};

// -----------------------------------------------------------------------------
// Closest point on triangle and point-triangle distance
// -----------------------------------------------------------------------------

/**
 * @brief Compute the closest point on triangle ABC to point P.
 *
 * Robustly handles all Voronoi regions (vertices/edges/interior).
 * Degenerate triangles fall back to nearest vertex.
 *
 * @param p Query point.
 * @param a Triangle vertex A.
 * @param b Triangle vertex B.
 * @param c Triangle vertex C.
 * @return Closest point on triangle to @p p.
 */
inline Vec3 closest_point_on_triangle(
  const Vec3 & p,
  const Vec3 & a,
  const Vec3 & b,
  const Vec3 & c)
{
  const Vec3 ab = b - a;
  const Vec3 ac = c - a;
  const Vec3 ap = p - a;

  float d1 = ab.dot(ap);
  float d2 = ac.dot(ap);
  if (d1 <= 0.0f && d2 <= 0.0f) {
    return a;
  }

  const Vec3 bp = p - b;
  float d3 = ab.dot(bp);
  float d4 = ac.dot(bp);
  if (d3 >= 0.0f && d4 <= d3) {
    return b;
  }

  float vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    float v = d1 / (d1 - d3);
    return a + v * ab;
  }

  const Vec3 cp = p - c;
  float d5 = ab.dot(cp);
  float d6 = ac.dot(cp);
  if (d6 >= 0.0f && d5 <= d6) {
    return c;
  }

  float vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    float w = d2 / (d2 - d6);
    return a + w * ac;
  }

  float va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return b + w * (c - b);
  }

  float ab2 = ab.dot(ab);
  float ac2 = ac.dot(ac);
  float abac = ab.dot(ac);
  float denom = (ab2 * ac2 - abac * abac);
  if (std::fabs(denom) < 1e-20f) {
    float da = (p - a).squaredNorm();
    float db = (p - b).squaredNorm();
    float dc = (p - c).squaredNorm();
    if (da <= db && da <= dc) {return a;}
    if (db <= dc) {return b;}
    return c;
  }
  float v = (ac2 * d1 - abac * d2) / denom;
  float w = (ab2 * d2 - abac * d1) / denom;
  return a + v * ab + w * ac;
}

/**
 * @brief Squared distance between point and triangle, with optional closest pt.
 *
 * @param p Query point.
 * @param a Triangle vertex A.
 * @param b Triangle vertex B.
 * @param c Triangle vertex C.
 * @param closest Optional output: closest point on triangle to @p p.
 * @return Squared Euclidean distance from @p p to triangle ABC.
 *
 * @note Squared distance avoids an extra sqrt and is preferred when
 *       comparing distances or using as thresholds.
 */
inline float point_triangle_squared_distance(
  const Vec3 & p,
  const Vec3 & a,
  const Vec3 & b,
  const Vec3 & c,
  Vec3 * closest = nullptr)
{
  Vec3 q = closest_point_on_triangle(p, a, b, c);
  if (closest) {*closest = q;}
  return (q - p).squaredNorm();
}

}  // namespace navmap

#endif  // NAVMAP_CORE__GEOMETRY_HPP
