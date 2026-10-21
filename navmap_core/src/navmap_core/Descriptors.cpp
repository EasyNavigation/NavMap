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

#include "navmap_core/Descriptors.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "Eigen/Eigenvalues"
#include "navmap_core/Iterators.hpp"

namespace navmap
{

void compute_descriptors(NavMap & nm, const DescriptorParams & p)
{
  const std::size_t n = nm.navcels.size();
  constexpr float kInf = std::numeric_limits<float>::infinity();
  TraversalScratch scratch;
  std::vector<Eigen::Vector3f> pts;

  if (p.slope) {
    auto v = nm.add_layer<float>("slope", "angle of the surface to the horizontal", "deg", 0.0f);
    for (NavCelId c = 0; c < n; ++c) {
      const float nz = std::clamp(std::abs(nm.navcels[c].normal.z()), 0.0f, 1.0f);
      v->data()[c] = std::acos(nz) * 180.0f / static_cast<float>(M_PI);
    }
    v->mark_all_changed();
  }

  if (p.roughness) {
    auto v = nm.add_layer<float>(
      "roughness", "RMS distance to the plane fitted around the NavCel", "m", 0.0f);
    for (NavCelId c = 0; c < n; ++c) {
      const Eigen::Vector3f center = nm.navcel_centroid(c);
      pts.clear();
      for_each_in_disc(
        nm, c, center, p.roughness_radius,
        [&](NavCelId x) {pts.push_back(nm.navcel_centroid(x));}, Metric::EUCLIDEAN, &scratch);
      float rms = 0.0f;
      if (pts.size() >= 3) {
        Eigen::Vector3f mean = Eigen::Vector3f::Zero();
        for (const auto & q : pts) {
          mean += q;
        }
        mean /= static_cast<float>(pts.size());
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (const auto & q : pts) {
          cov += (q - mean) * (q - mean).transpose();
        }
        // The smallest eigenvalue is the summed squared distance to the best plane
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov, Eigen::EigenvaluesOnly);
        rms = std::sqrt(std::max(0.0f, es.eigenvalues()[0]) / static_cast<float>(pts.size()));
      }
      v->data()[c] = rms;
    }
    v->mark_all_changed();
  }

  if (p.step) {
    auto v = nm.add_layer<float>(
      "step", "height span of the surface around the NavCel", "m", 0.0f);
    for (NavCelId c = 0; c < n; ++c) {
      float lo = kInf, hi = -kInf;
      for_each_in_disc(
        nm, c, nm.navcel_centroid(c), p.step_radius,
        [&](NavCelId x) {
          for (const auto vid : nm.navcels[x].v) {
            const float z = nm.positions.z[vid];
            lo = std::min(lo, z);
            hi = std::max(hi, z);
          }
        }, Metric::EUCLIDEAN, &scratch);
      v->data()[c] = hi - lo;
    }
    v->mark_all_changed();
  }

  if (p.clearance) {
    auto v = nm.add_layer<float>(
      "clearance", "free height to the next surface above", "m", kInf);
    const Eigen::Vector3f up(0.0f, 0.0f, 1.0f);
    for (NavCelId c = 0; c < n; ++c) {
      const Eigen::Vector3f o = nm.navcel_centroid(c) + Eigen::Vector3f(0.0f, 0.0f, 1e-3f);
      NavCelId hit_cid;
      float t;
      Eigen::Vector3f hit;
      const bool hit_any = nm.raycast(o, up, hit_cid, t, hit) && hit_cid != c;
      v->data()[c] = hit_any && t + 1e-3f <= p.clearance_max ? t + 1e-3f : kInf;
    }
    v->mark_all_changed();
  }
}

}  // namespace navmap
