// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3.0, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.


#include <gtest/gtest.h>
#include <cstdint>
#include <vector>
#include <string>

#include "navmap_core/NavMap.hpp"

using namespace navmap;

namespace
{

// Build a 1-triangle map with layers "occ" (U8) and "cost" (F32)
void fill_one_tri_map(NavMap & m)
{
  const uint32_t i0 = m.add_vertex(Eigen::Vector3f(0.f, 0.f, 0.f));
  const uint32_t i1 = m.add_vertex(Eigen::Vector3f(1.f, 0.f, 0.f));
  const uint32_t i2 = m.add_vertex(Eigen::Vector3f(0.f, 1.f, 0.f));

  (void)m.add_navcel(i0, i1, i2);

  // U8 layer "occ"
  {
    auto occ = m.layers.add_or_get<uint8_t>("occ", 1, layer_type_tag<uint8_t>());
    auto & v = occ->mutable_data();
    v.resize(1);
    v[0] = 254; // occupied (black)
    m.layer_meta["occ"].description = "occupancy";
    m.layer_meta["occ"].unit = "";
    m.layer_meta["occ"].per_cell = true;
  }

  // F32 layer "cost"
  {
    auto cost = m.layers.add_or_get<float>("cost", 1, layer_type_tag<float>());
    auto & v = cost->mutable_data();
    v.resize(1);
    v[0] = 10.0f;
    m.layer_meta["cost"].description = "traversal cost";
    m.layer_meta["cost"].unit = "";
    m.layer_meta["cost"].per_cell = true;
  }
}

// Same geometry as above but different layer values + an extra layer
void fill_one_tri_map_variant(NavMap & m)
{
  fill_one_tri_map(m);

  if (auto occ = std::dynamic_pointer_cast<LayerView<uint8_t>>(m.layers.get("occ"))) {
    auto & v = occ->mutable_data();
    v[0] = 0; // free (white)
  }
  if (auto cost = std::dynamic_pointer_cast<LayerView<float>>(m.layers.get("cost"))) {
    auto & v = cost->mutable_data();
    v[0] = 5.0f;
  }

  // Extra layer only here
  {
    auto extra = m.layers.add_or_get<double>("extra_only_here", 1, layer_type_tag<double>());
    auto & v = extra->mutable_data();
    v.resize(1);
    v[0] = 123.0;
    m.layer_meta["extra_only_here"].description = "extra";
    m.layer_meta["extra_only_here"].unit = "";
    m.layer_meta["extra_only_here"].per_cell = true;
  }
}

} // namespace

TEST(NavMapAssign, CopiesOnlyLayersWhenGeometryEqualAndRemovesExtras)
{
  NavMap src; fill_one_tri_map(src);
  NavMap dst; fill_one_tri_map_variant(dst); // has extra layer and different values

  // Keep destination geometry pointers to verify geometry is not rebuilt
  const float * px = dst.positions.x.data();
  const float * py = dst.positions.y.data();
  const float * pz = dst.positions.z.data();
  const std::uint32_t * tri0 = &dst.navcels[0].v[0];

  // Optimized assignment (equal geometry): only layers/metadata should change
  dst = src;

  // Geometry untouched (same buffers)
  EXPECT_EQ(px, dst.positions.x.data());
  EXPECT_EQ(py, dst.positions.y.data());
  EXPECT_EQ(pz, dst.positions.z.data());
  EXPECT_EQ(tri0, &dst.navcels[0].v[0]);

  // Extra layer removed
  EXPECT_FALSE(static_cast<bool>(dst.layers.get("extra_only_here")));

  // Layers equalized
  {
    auto occ_s = std::dynamic_pointer_cast<LayerView<uint8_t>>(src.layers.get("occ"));
    auto occ_d = std::dynamic_pointer_cast<LayerView<uint8_t>>(dst.layers.get("occ"));
    ASSERT_TRUE(occ_s && occ_d);
    EXPECT_EQ(occ_s->data().size(), occ_d->data().size());
    EXPECT_EQ(occ_s->data()[0], occ_d->data()[0]);
  }
  {
    auto c_s = std::dynamic_pointer_cast<LayerView<float>>(src.layers.get("cost"));
    auto c_d = std::dynamic_pointer_cast<LayerView<float>>(dst.layers.get("cost"));
    ASSERT_TRUE(c_s && c_d);
    EXPECT_EQ(c_s->data().size(), c_d->data().size());
    EXPECT_FLOAT_EQ(c_s->data()[0], c_d->data()[0]);
  }

  // Metadata synchronized (compare fields, not whole struct)
  ASSERT_TRUE(dst.layer_meta.count("occ") && src.layer_meta.count("occ"));
  EXPECT_EQ(dst.layer_meta.at("occ").description, src.layer_meta.at("occ").description);
  EXPECT_EQ(dst.layer_meta.at("occ").unit, src.layer_meta.at("occ").unit);
  EXPECT_EQ(dst.layer_meta.at("occ").per_cell, src.layer_meta.at("occ").per_cell);
}

TEST(NavMapAssign, FullCopyWhenGeometryDifferent)
{
  NavMap src; fill_one_tri_map(src);
  NavMap dst;

  // Make destination geometry different (2 triangles)
  const uint32_t a = dst.add_vertex(Eigen::Vector3f(0, 0, 0));
  const uint32_t b = dst.add_vertex(Eigen::Vector3f(1, 0, 0));
  const uint32_t c = dst.add_vertex(Eigen::Vector3f(0, 1, 0));
  const uint32_t d = dst.add_vertex(Eigen::Vector3f(1, 1, 0));

  (void)dst.add_navcel(a, b, c);
  (void)dst.add_navcel(b, d, c);

  // Record sizes to assert they change to match source
  const std::size_t dst_vertices_before = dst.positions.x.size();
  const std::size_t dst_tris_before = dst.navcels.size();

  // Different geometry -> full copy expected
  dst = src;

  // Sizes must now match src (this guarantees geometry was rewritten)
  EXPECT_EQ(dst.positions.x.size(), src.positions.x.size());  // 3
  EXPECT_EQ(dst.positions.y.size(), src.positions.y.size());
  EXPECT_EQ(dst.positions.z.size(), src.positions.z.size());
  EXPECT_EQ(dst.navcels.size(), src.navcels.size());          // 1

  // And sizes must have changed from the "different geometry" we set up
  EXPECT_NE(dst_vertices_before, dst.positions.x.size());
  EXPECT_NE(dst_tris_before, dst.navcels.size());

  // Optional: exact geometry content equality
  for (std::size_t i = 0; i < src.positions.x.size(); ++i) {
    EXPECT_FLOAT_EQ(dst.positions.x[i], src.positions.x[i]);
    EXPECT_FLOAT_EQ(dst.positions.y[i], src.positions.y[i]);
    EXPECT_FLOAT_EQ(dst.positions.z[i], src.positions.z[i]);
  }
  for (std::size_t t = 0; t < src.navcels.size(); ++t) {
    EXPECT_EQ(dst.navcels[t].v[0], src.navcels[t].v[0]);
    EXPECT_EQ(dst.navcels[t].v[1], src.navcels[t].v[1]);
    EXPECT_EQ(dst.navcels[t].v[2], src.navcels[t].v[2]);
  }

  // Or simply: fast equality check via fingerprint
  EXPECT_TRUE(dst.has_same_geometry(src));

  // Layers exist after full copy
  auto occ = std::dynamic_pointer_cast<LayerView<uint8_t>>(dst.layers.get("occ"));
  auto cost = std::dynamic_pointer_cast<LayerView<float>>(dst.layers.get("cost"));
  ASSERT_TRUE(occ && cost);
  EXPECT_EQ(occ->data().size(), 1u);
  EXPECT_EQ(cost->data().size(), 1u);
}

TEST(NavMapAssign, HashSkipsCopyWhenLayerIdentical)
{
  NavMap src; fill_one_tri_map(src);
  NavMap dst; fill_one_tri_map(dst);

  // Track destination "occ" buffer address
  auto occ_dst = std::dynamic_pointer_cast<LayerView<uint8_t>>(dst.layers.get("occ"));
  ASSERT_TRUE(occ_dst);
  const uint8_t * occ_ptr_before = occ_dst->data().data();

  // Since size and content_hash are equal, assignment should not trigger set_data()
  dst = src;

  auto occ_dst_after = std::dynamic_pointer_cast<LayerView<uint8_t>>(dst.layers.get("occ"));
  ASSERT_TRUE(occ_dst_after);
  const uint8_t * occ_ptr_after = occ_dst_after->data().data();
  EXPECT_EQ(occ_ptr_before, occ_ptr_after); // same backing storage

  // Content must still be equal
  auto occ_src = std::dynamic_pointer_cast<LayerView<uint8_t>>(src.layers.get("occ"));
  ASSERT_TRUE(occ_src);
  EXPECT_EQ(occ_src->data()[0], occ_dst_after->data()[0]);
}

TEST(NavMapAssign, RemoveExtraLayersOnEqualGeometry)
{
  NavMap src; fill_one_tri_map(src);
  NavMap dst; fill_one_tri_map(dst);

  // Extra layer only in destination
  {
    auto extra = dst.layers.add_or_get<double>("extra_only_here", 1, layer_type_tag<double>());
    auto & v = extra->mutable_data();
    v.resize(1);
    v[0] = 42.0;
    dst.layer_meta["extra_only_here"].description = "extra";
    dst.layer_meta["extra_only_here"].unit = "";
    dst.layer_meta["extra_only_here"].per_cell = true;
  }

  // Equal geometry -> assignment must remove the extra layer
  dst = src;
  EXPECT_FALSE(static_cast<bool>(dst.layers.get("extra_only_here")));
}

TEST(NavMapAssign, LayerMetaSynchronized)
{
  NavMap src; fill_one_tri_map(src);
  NavMap dst; fill_one_tri_map(dst);

  // Change metadata in source
  src.layer_meta["occ"].description = "changed by src";
  src.layer_meta["occ"].unit = "unitless";
  src.layer_meta["occ"].per_cell = true;

  dst = src;

  ASSERT_TRUE(dst.layer_meta.count("occ"));
  EXPECT_EQ(dst.layer_meta.at("occ").description, "changed by src");
  EXPECT_EQ(dst.layer_meta.at("occ").unit, "unitless");
  EXPECT_TRUE(dst.layer_meta.at("occ").per_cell);
}
