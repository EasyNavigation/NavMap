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

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <limits>
#include "navmap_core/Geometry.hpp"

using namespace navmap;

static constexpr float kEps = 1e-6f;


TEST(Geometry_TriangleArea, BasicScaledDegenerate) {
  Vec3 a(0.0f, 0.0f, 0.0f);
  Vec3 b(1.0f, 0.0f, 0.0f);
  Vec3 c(0.0f, 1.0f, 0.0f);
  EXPECT_NEAR(triangle_area(a, b, c), 0.5f, kEps);

  Vec3 a2(0.0f, 0.0f, 0.0f);
  Vec3 b2(2.0f, 0.0f, 0.0f);
  Vec3 c2(0.0f, 3.0f, 0.0f);
  EXPECT_NEAR(triangle_area(a2, b2, c2), 3.0f, kEps);

  // Colinear -> area 0
  Vec3 p0(0.0f, 0.0f, 0.0f);
  Vec3 p1(1.0f, 1.0f, 1.0f);
  Vec3 p2(2.0f, 2.0f, 2.0f);
  EXPECT_NEAR(triangle_area(p0, p1, p2), 0.0f, kEps);
}

TEST(Geometry_TriangleNormal, OrientationAndDegenerate) {
  // XY plane, CCW -> +Z
  Vec3 a(0.0f, 0.0f, 0.0f);
  Vec3 b(1.0f, 0.0f, 0.0f);
  Vec3 c(0.0f, 1.0f, 0.0f);
  Vec3 n1 = triangle_normal(a, b, c);
  EXPECT_NEAR(n1.x(), 0.0f, kEps);
  EXPECT_NEAR(n1.y(), 0.0f, kEps);
  EXPECT_NEAR(n1.z(), 1.0f, kEps);
  EXPECT_NEAR(n1.norm(), 1.0f, kEps);

  // XY plane, CW -> -Z
  Vec3 n2 = triangle_normal(a, c, b);
  EXPECT_NEAR(n2.x(), 0.0f, kEps);
  EXPECT_NEAR(n2.y(), 0.0f, kEps);
  EXPECT_NEAR(n2.z(), -1.0f, kEps);
  EXPECT_NEAR(n2.norm(), 1.0f, kEps);

  // XZ plane, (a,b,c): (1,0,0) x (0,0,1) -> -Y
  Vec3 axz(0.0f, 0.0f, 0.0f);
  Vec3 bxz(1.0f, 0.0f, 0.0f);
  Vec3 cxz(0.0f, 0.0f, 1.0f);
  Vec3 nxz = triangle_normal(axz, bxz, cxz);
  EXPECT_NEAR(nxz.x(), 0.0f, kEps);
  EXPECT_NEAR(nxz.y(), -1.0f, kEps);
  EXPECT_NEAR(nxz.z(), 0.0f, kEps);

  // XZ plane, reversed winding -> +Y
  Vec3 nxz_inv = triangle_normal(axz, cxz, bxz);
  EXPECT_NEAR(nxz_inv.x(), 0.0f, kEps);
  EXPECT_NEAR(nxz_inv.y(), 1.0f, kEps);
  EXPECT_NEAR(nxz_inv.z(), 0.0f, kEps);

  // YZ plane, CCW -> +X
  Vec3 ayz(0.0f, 0.0f, 0.0f);
  Vec3 byz(0.0f, 1.0f, 0.0f);
  Vec3 cyz(0.0f, 0.0f, 1.0f);
  Vec3 nyz = triangle_normal(ayz, byz, cyz);
  EXPECT_NEAR(nyz.x(), 1.0f, kEps);
  EXPECT_NEAR(nyz.y(), 0.0f, kEps);
  EXPECT_NEAR(nyz.z(), 0.0f, kEps);

  // Tilted triangle: expect a non-axis-aligned unit normal.
  Vec3 at(0.0f, 0.0f, 0.0f);
  Vec3 bt(1.0f, 0.0f, 1.0f);
  Vec3 ct(0.0f, 1.0f, 0.5f);
  Vec3 nt = triangle_normal(at, bt, ct);
  EXPECT_NEAR(nt.norm(), 1.0f, kEps);
  // Components should not match a pure axis normal.
  EXPECT_GT(std::abs(nt.x()), 0.0f);
  EXPECT_GT(std::abs(nt.y()), 0.0f);
  EXPECT_GT(std::abs(nt.z()), 0.0f);

  // Degenerate: colinear -> fallback (0, 0, 1).
  Vec3 d0(0.0f, 0.0f, 0.0f);
  Vec3 d1(1.0f, 1.0f, 1.0f);
  Vec3 d2(2.0f, 2.0f, 2.0f);
  Vec3 nd = triangle_normal(d0, d1, d2);
  EXPECT_NEAR(nd.x(), 0.0f, kEps);
  EXPECT_NEAR(nd.y(), 0.0f, kEps);
  EXPECT_NEAR(nd.z(), 1.0f, kEps);
}

TEST(Geometry_RayTriangle, HitMissEdgeBackfaceParallel) {
  Vec3 a(0.0f, 0.0f, 0.0f);
  Vec3 b(1.0f, 0.0f, 0.0f);
  Vec3 c(0.0f, 1.0f, 0.0f);

  // Hit: vertical ray downward.
  Vec3 o(0.25f, 0.25f, 1.0f);
  Vec3 d(0.0f, 0.0f, -1.0f);
  float t = 0.0f;
  float u = 0.0f;
  float v = 0.0f;
  ASSERT_TRUE(ray_triangle_intersect(o, d, a, b, c, t, u, v));
  EXPECT_NEAR(t, 1.0f, 1e-5f);
  Vec3 hit = o + t * d;
  EXPECT_NEAR(hit.z(), 0.0f, kEps);
  float w = 1.0f - u - v;
  EXPECT_GE(u, -1e-5f);
  EXPECT_LE(u, 1.0f + 1e-5f);
  EXPECT_GE(v, -1e-5f);
  EXPECT_LE(v, 1.0f + 1e-5f);
  EXPECT_GE(w, -1e-5f);
  EXPECT_LE(w, 1.0f + 1e-5f);

  // Miss: ray passes over triangle footprint but outside in XY.
  Vec3 o_miss(1.5f, 1.5f, 1.0f);
  ASSERT_FALSE(ray_triangle_intersect(o_miss, d, a, b, c, t, u, v));

  // Edge hit: along edge (a-b).
  Vec3 o_edge(0.5f, 0.0f, 1.0f);
  ASSERT_TRUE(ray_triangle_intersect(o_edge, d, a, b, c, t, u, v));
  hit = o_edge + t * d;
  EXPECT_NEAR(hit.x(), 0.5f, 1e-5f);
  EXPECT_NEAR(hit.y(), 0.0f, 1e-5f);
  EXPECT_NEAR(hit.z(), 0.0f, 1e-5f);

  // Backface: same geometry with reversed winding; algorithm does no culling.
  ASSERT_TRUE(ray_triangle_intersect(o, d, a, c, b, t, u, v));

  // Parallel upward (t < 0) -> no valid intersection.
  Vec3 d_up(0.0f, 0.0f, 1.0f);
  ASSERT_FALSE(ray_triangle_intersect(o, d_up, a, b, c, t, u, v));
}

TEST(Geometry_AABB, ExpandPointBoxAxis) {
  AABB box;

  // After first point, min == max == p1.
  Vec3 p1(0.0f, 0.0f, 0.0f);
  box.expand(p1);
  EXPECT_NEAR(box.min.x(), 0.0f, kEps);
  EXPECT_NEAR(box.min.y(), 0.0f, kEps);
  EXPECT_NEAR(box.min.z(), 0.0f, kEps);
  EXPECT_NEAR(box.max.x(), 0.0f, kEps);
  EXPECT_NEAR(box.max.y(), 0.0f, kEps);
  EXPECT_NEAR(box.max.z(), 0.0f, kEps);

  // Expand with a second point.
  Vec3 p2(1.0f, 2.0f, 3.0f);
  box.expand(p2);
  EXPECT_NEAR(box.min.x(), 0.0f, kEps);
  EXPECT_NEAR(box.min.y(), 0.0f, kEps);
  EXPECT_NEAR(box.min.z(), 0.0f, kEps);
  EXPECT_NEAR(box.max.x(), 1.0f, kEps);
  EXPECT_NEAR(box.max.y(), 2.0f, kEps);
  EXPECT_NEAR(box.max.z(), 3.0f, kEps);

  // Expand with another box.
  AABB b2;
  b2.expand(Vec3(-1.0f, 5.0f, -2.0f));
  b2.expand(Vec3(0.5f, 6.0f, 4.0f));
  box.expand(b2);
  EXPECT_NEAR(box.min.x(), -1.0f, kEps);
  EXPECT_NEAR(box.min.y(), 0.0f, kEps);
  EXPECT_NEAR(box.min.z(), -2.0f, kEps);
  EXPECT_NEAR(box.max.x(), 1.0f, kEps);
  EXPECT_NEAR(box.max.y(), 6.0f, kEps);
  EXPECT_NEAR(box.max.z(), 4.0f, kEps);

  // Longest axis is either Y or Z (tie allowed by implementation).
  int axis = box.longest_axis();
  EXPECT_TRUE(axis == 1 || axis == 2);
}

TEST(Geometry_AABB, IntersectsRayVariants) {
  // Unit box [0,1]^3.
  AABB box;
  box.expand(Vec3(0.0f, 0.0f, 0.0f));
  box.expand(Vec3(1.0f, 1.0f, 1.0f));

  // Ray from (-1, 0.5, 0.5) along +X -> intersects.
  Vec3 o1(-1.0f, 0.5f, 0.5f);
  Vec3 d1(1.0f, 0.0f, 0.0f);
  EXPECT_TRUE(box.intersects_ray(o1, d1));

  // Ray from (2, 0.5, 0.5) along +X -> misses.
  Vec3 o2(2.0f, 0.5f, 0.5f);
  Vec3 d2(1.0f, 0.0f, 0.0f);
  EXPECT_FALSE(box.intersects_ray(o2, d2));

  // Ray starting inside -> intersects.
  Vec3 o3(0.5f, 0.5f, 0.5f);
  Vec3 d3(1.0f, 0.0f, 0.0f);
  EXPECT_TRUE(box.intersects_ray(o3, d3));

  // Ray parallel to Y planes and outside -> misses.
  Vec3 o4(0.5f, 2.0f, 0.5f);
  Vec3 d4(1.0f, 0.0f, 0.0f);
  EXPECT_FALSE(box.intersects_ray(o4, d4));

  // Oblique ray crossing the box.
  Vec3 o5(-1.0f, -1.0f, -1.0f);
  Vec3 d5(1.0f, 1.0f, 1.0f);
  EXPECT_TRUE(box.intersects_ray(o5, d5));

  // Small tmax can prevent intersection.
  EXPECT_FALSE(box.intersects_ray(o1, d1, 0.1f));
}

TEST(Geometry_AABB, ContainsXYWithZEps) {
  AABB box;
  box.expand(Vec3(0.0f, 0.0f, 0.0f));
  box.expand(Vec3(1.0f, 1.0f, 0.0f));

  Vec3 p_above(0.5f, 0.5f, 0.4f);
  EXPECT_TRUE(box.contains_xy(p_above, 0.5f));
  EXPECT_FALSE(box.contains_xy(p_above, 0.1f));

  Vec3 p_on(0.2f, 0.8f, 0.0f);
  EXPECT_TRUE(box.contains_xy(p_on, 0.0f));

  Vec3 p_out(1.2f, 0.5f, 0.0f);
  EXPECT_FALSE(box.contains_xy(p_out, 0.5f));
}
