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

// Micro-benchmark of the NavMap core queries over grid maps of growing size.
// Usage: navmap_bench [max_side_cells=1000] [floors=1]

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "navmap_core/DistanceField.hpp"
#include "navmap_core/Iterators.hpp"
#include "navmap_core/NavMap.hpp"

using navmap::NavCelId;
using navmap::NavMap;

namespace
{

using Clock = std::chrono::steady_clock;

// side x side cells of res metres per floor, floors 3 m apart.
NavMap make_map(int side, int floors, float res)
{
  NavMap nm;
  for (int f = 0; f < floors; ++f) {
    const float z = 3.0f * static_cast<float>(f);
    const std::size_t s = nm.create_surface("map");
    const auto base = static_cast<uint32_t>(nm.positions.size());
    for (int j = 0; j <= side; ++j) {
      for (int i = 0; i <= side; ++i) {
        nm.add_vertex({i * res, j * res, z});
      }
    }
    auto v = [&](int i, int j) {return base + static_cast<uint32_t>(j * (side + 1) + i);};
    for (int j = 0; j < side; ++j) {
      for (int i = 0; i < side; ++i) {
        nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j), v(i + 1, j + 1)));
        nm.add_navcel_to_surface(s, nm.add_navcel(v(i, j), v(i + 1, j + 1), v(i, j + 1)));
      }
    }
  }
  return nm;
}

// Median time per call in microseconds over `reps` batches of `n` calls.
template<typename F>
double median_us(int reps, int n, F && f)
{
  std::vector<double> t;
  for (int r = 0; r < reps; ++r) {
    const auto t0 = Clock::now();
    for (int k = 0; k < n; ++k) {
      f(k);
    }
    t.push_back(std::chrono::duration<double, std::micro>(Clock::now() - t0).count() / n);
  }
  std::nth_element(t.begin(), t.begin() + t.size() / 2, t.end());
  return t[t.size() / 2];
}

std::size_t geometry_bytes(const NavMap & nm)
{
  std::size_t b = nm.positions.size() * 3 * sizeof(float) + nm.navcels.size() *
    sizeof(navmap::NavCel);
  for (const auto & s : nm.surfaces) {
    b += s.bvh.size() * sizeof(navmap::BVHNode) + (s.prim_indices.size() + s.navcels.size()) * 4;
  }
  return b;
}

}  // namespace

int main(int argc, char ** argv)
{
  const int max_side = argc > 1 ? std::atoi(argv[1]) : 1000;
  const int floors = argc > 2 ? std::atoi(argv[2]) : 1;
  const float res = 0.05f;
  const int N = 20000, R = 7;

  std::printf(
    "floors=%d res=%.2f m, median us/op over %d x %d calls\n"
    "%9s %9s %9s %9s %9s %9s %9s %9s %9s %9s %9s\n",
    floors, res, R, N, "navcels", "MB", "build_ms", "loc_nohnt", "loc_hint", "ray_down",
    "ray_rand", "closest", "area_r0.5", "copy_same", "copy_deep");

  for (int side : {100, 300, 1000, 3000}) {
    if (side > max_side) {break;}
    NavMap nm = make_map(side, floors, res);
    const auto t0 = Clock::now();
    nm.rebuild_geometry_accels();
    const double build_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    nm.add_layer<uint8_t>("occupancy", "", "", 0);

    const float extent = side * res;
    std::mt19937 rng(1);
    std::uniform_real_distribution<float> U(0.01f, extent - 0.01f);
    std::uniform_int_distribution<int> F(0, floors - 1);
    std::normal_distribution<float> G(0.0f, 1.0f);
    std::vector<Eigen::Vector3f> pts(N), dirs(N);
    for (int k = 0; k < N; ++k) {
      pts[k] = {U(rng), U(rng), 3.0f * static_cast<float>(F(rng)) + 0.1f};
      dirs[k] = Eigen::Vector3f(G(rng), G(rng), G(rng)).normalized();
    }

    std::size_t s; NavCelId c; Eigen::Vector3f bary, hit, cp; float t, sq;
    const double loc_nohint = median_us(
      R, N, [&](int k) {nm.locate_navcel(pts[k], s, c, bary, &hit);});

    // Coherent trajectory: small steps reusing the previous cell as hint.
    NavMap::LocateOpts opts;
    nm.locate_navcel(pts[0], s, c, bary, &hit);
    Eigen::Vector3f p = pts[0];
    const double loc_hint = median_us(
      R, N, [&](int k) {
        p.x() = std::clamp(p.x() + 0.02f * ((k % 7) - 3), 0.01f, extent - 0.01f);
        p.y() = std::clamp(p.y() + 0.02f * ((k % 5) - 2), 0.01f, extent - 0.01f);
        opts.hint_cid = c;
        nm.locate_navcel(p, s, c, bary, &hit, opts);
      });

    const double ray_down = median_us(
      R, N, [&](int k) {
        nm.raycast(pts[k] + Eigen::Vector3f(0, 0, 1), {0.0f, 0.0f, -1.0f}, c, t, hit);
      });
    const double ray_rand = median_us(
      R, N, [&](int k) {nm.raycast(pts[k], dirs[k], c, t, hit);});
    const double closest = median_us(
      R, N, [&](int k) {nm.closest_navcel(pts[k] + 0.3f * dirs[k], s, c, cp, sq);});
    const double area = median_us(
      R, N / 20, [&](int k) {
        nm.set_area<uint8_t>(pts[k], 254, "occupancy", navmap::AreaShape::CIRCULAR, 0.5f);
      });

    NavMap same = nm;
    const double copy_same = median_us(R, 10, [&](int) {same = nm;});
    const double copy_deep = median_us(R, 10, [&](int) {NavMap d(nm); (void)d;});

    std::printf(
      "%9zu %9.1f %9.1f %9.3f %9.3f %9.3f %9.3f %9.3f %9.1f %9.0f %9.0f\n",
      nm.navcels.size(), geometry_bytes(nm) / 1e6, build_ms, loc_nohint, loc_hint, ray_down,
      ray_rand, closest, area, copy_same, copy_deep);
  }

  // Iterators (with a reused scratch), from a located seed; us per call
  std::printf(
    "\niterators, us/call (cells visited)\n%9s %16s %16s %16s %16s %16s %16s %16s %16s\n",
    "navcels", "disc_e_r0.5", "disc_e_r2", "disc_g_r0.5", "disc_g_r2", "aabb_2x2", "poly_1x1",
    "segment_5m", "bydist_r2");
  for (int side : {100, 300, 1000, 3000}) {
    if (side > max_side) {break;}
    NavMap nm = make_map(side, floors, res);
    nm.rebuild_geometry_accels();
    const float extent = side * res;
    std::mt19937 rng(2);
    std::uniform_real_distribution<float> U(2.5f, std::max(2.6f, extent - 2.5f));
    const int M = 2000;
    std::vector<Eigen::Vector3f> pts(M);
    std::vector<NavCelId> seeds(M);
    for (int k = 0; k < M; ++k) {
      pts[k] = {U(rng), U(rng), 0.1f};
      std::size_t s; Eigen::Vector3f bary, hit;
      nm.locate_navcel(pts[k], s, seeds[k], bary, &hit);
    }
    navmap::TraversalScratch sc;
    std::size_t cells = 0;
    auto count = [&](NavCelId) {++cells;};
    char col[8][32];
    int ci = 0;
    auto run = [&](int n, auto && body) {
        cells = 0;
        const double us = median_us(R, n, body);
        std::snprintf(col[ci++], sizeof(col[0]), "%.1f (%zu)", us, cells / (R * n));
      };
    run(M, [&](int k) {
        navmap::for_each_in_disc(nm, seeds[k], pts[k], 0.5f, count,
        navmap::Metric::EUCLIDEAN, &sc);
                                        });
    run(M / 10, [&](int k) {
        navmap::for_each_in_disc(nm, seeds[k], pts[k], 2.0f, count,
        navmap::Metric::EUCLIDEAN, &sc);
                                        });
    run(M, [&](int k) {
        navmap::for_each_in_disc(nm, seeds[k], pts[k], 0.5f, count,
        navmap::Metric::GEODESIC, &sc);
                                       });
    run(M / 10, [&](int k) {
        navmap::for_each_in_disc(nm, seeds[k], pts[k], 2.0f, count,
        navmap::Metric::GEODESIC, &sc);
                                       });
    run(M, [&](int k) {
        navmap::AABB b;
        b.expand(pts[k] - Eigen::Vector3f(1, 1, 1));
        b.expand(pts[k] + Eigen::Vector3f(1, 1, 1));
        navmap::for_each_in_aabb(nm, b, count, &sc);
      });
    run(M, [&](int k) {
        const float x = pts[k].x(), y = pts[k].y();
        navmap::for_each_in_polygon(
          nm, seeds[k], {{x - 0.5f, y - 0.5f}, {x + 0.5f, y - 0.5f}, {x + 0.5f, y + 0.5f},
            {x - 0.5f, y + 0.5f}}, count, &sc);
      });
    run(M, [&](int k) {
        navmap::for_each_on_segment(nm, seeds[k], pts[k],
        pts[(k + 1) % M].head<2>().isZero() ? pts[k] :
        Eigen::Vector3f(pts[k] + 5.0f * (pts[(k + 1) % M] - pts[k]).normalized()),
        [&](NavCelId, float, float) {++cells;});
      });
    run(M / 10, [&](int k) {
        navmap::for_each_by_distance(nm, seeds[k], pts[k], 2.0f,
        [&](NavCelId, float) {++cells;}, &sc);
                                              });
    std::printf("%9zu", nm.navcels.size());
    for (auto & c : col) {
      std::printf(" %16s", c);
    }
    std::printf("\n");
  }

  // Distance fields (borders + obstacle cells as seeds), radius r: full computation with
  // NavMap::distance_field (nearest-feature propagation) and DistanceField (exact), and
  // incremental updates of DistanceField (k obstacle cells moved), in ms.
  std::printf(
    "\ndistance fields, ms\n%9s %5s %14s %14s %14s %14s %14s\n", "navcels", "r",
    "propag_full", "exact_full", "upd_1", "upd_10", "upd_100");
  for (int side : {100, 300, 1000}) {
    if (side > max_side) {break;}
    NavMap nm = make_map(side, 1, res);
    nm.rebuild_geometry_accels();
    std::mt19937 rng(5);
    std::uniform_int_distribution<NavCelId> C(0, static_cast<NavCelId>(nm.navcels.size() - 1));
    std::vector<NavCelId> obstacles(nm.navcels.size() / 50);
    for (auto & o : obstacles) {
      o = C(rng);
    }
    for (const float r : {0.7f, 2.0f}) {
      auto ms = [](auto && f) {
          const auto t0 = Clock::now();
          f();
          return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        };
      const double propag = ms([&] {(void)nm.distance_field(obstacles, true, r);});
      navmap::DistanceField df(nm, true, r);
      const double exact = ms([&] {df.compute(obstacles);});
      double upd[3];
      int i = 0;
      for (const int k : {1, 10, 100}) {
        std::vector<NavCelId> rem(obstacles.begin(), obstacles.begin() + k), add(k);
        for (auto & a : add) {
          a = C(rng);
        }
        upd[i++] = ms([&] {df.update(add, rem);});
        df.update(rem, add);  // back
      }
      std::printf(
        "%9zu %5.1f %14.2f %14.2f %14.3f %14.3f %14.3f\n", nm.navcels.size(), r, propag, exact,
        upd[0], upd[1], upd[2]);
    }
  }
  return 0;
}
