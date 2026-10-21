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

#include "navmap_core/Combine.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <type_traits>

namespace navmap
{

namespace
{

// Calls f(LayerView<T>&) with the typed view of @p base; false if its type is not supported.
template<class F>
bool with_view(const std::shared_ptr<LayerViewBase> & base, F && f)
{
  if (auto v = std::dynamic_pointer_cast<LayerView<uint8_t>>(base)) {f(*v); return true;}
  if (auto v = std::dynamic_pointer_cast<LayerView<float>>(base)) {f(*v); return true;}
  if (auto v = std::dynamic_pointer_cast<LayerView<double>>(base)) {f(*v); return true;}
  if (auto v = std::dynamic_pointer_cast<LayerView<int16_t>>(base)) {f(*v); return true;}
  return false;
}

bool is_unk(double v, const std::optional<double> & u)
{
  if (!u) {return false;}
  return std::isnan(*u) ? std::isnan(v) : v == *u;
}

// Value as stored in a layer of type T (rounded and clamped as NavMap::layer_get); nullopt if T
// cannot hold it (NaN into an integer).
template<class T>
std::optional<T> to_type(double v)
{
  if constexpr (std::is_floating_point_v<T>) {
    return static_cast<T>(v);
  } else {
    if (std::isnan(v)) {return std::nullopt;}
    const double lo = static_cast<double>(std::numeric_limits<T>::lowest());
    const double hi = static_cast<double>(std::numeric_limits<T>::max());
    return static_cast<T>(std::llround(std::clamp(v, lo, hi)));
  }
}

template<class T>
bool same(T a, T b)
{
  if constexpr (std::is_floating_point_v<T>) {
    if (std::isnan(a) && std::isnan(b)) {return true;}
  }
  return a == b;
}

template<class D, class S>
std::ptrdiff_t run(
  LayerView<D> & dst, const LayerView<S> & src, const std::optional<double> & dst_unk,
  const std::optional<double> & src_unk, CombineOp op, const CombineOptions & o,
  LayerView<uint8_t> * prov)
{
  std::ptrdiff_t changed = 0;
  auto & d = dst.data();
  const auto & s = src.data();
  auto one = [&](NavCelId c) {
      if (c >= d.size()) {return;}
      const double sv = static_cast<double>(s[c]);
      const double dv = static_cast<double>(d[c]);
      const bool su = is_unk(sv, src_unk), du = is_unk(dv, dst_unk);
      double nv;
      bool from_src = true;  // the result is the source's decision
      switch (op) {
        case CombineOp::TRUE_OVERWRITE:
          if (su && !dst_unk) {return;}
          nv = su ? *dst_unk : sv;
          break;
        case CombineOp::OVERWRITE:
          if (su) {return;}
          nv = sv;
          break;
        case CombineOp::MAX:
        case CombineOp::MAX_KEEP_UNKNOWN:
        case CombineOp::MIN:
          if (su || (du && op == CombineOp::MAX_KEEP_UNKNOWN)) {return;}
          if (du) {
            nv = sv;
          } else {
            nv = op == CombineOp::MIN ? std::min(dv, sv) : std::max(dv, sv);
            from_src = nv == sv;
          }
          break;
        case CombineOp::ADD:
          if (su) {return;}
          nv = std::min(du ? sv : dv + sv, o.cap);
          from_src = du || sv != 0.0;
          break;
        default:
          return;
      }
      const auto t = to_type<D>(nv);
      if (!t) {return;}
      if (!same(*t, d[c])) {
        d[c] = *t;
        dst.mark_changed(c);
        ++changed;
      }
      if (prov && from_src && (*prov)[c] != o.source_id) {
        (*prov)[c] = o.source_id;
        prov->mark_changed(c);
      }
    };
  if (o.cells.empty()) {
    for (NavCelId c = 0; c < d.size(); ++c) {
      one(c);
    }
  } else {
    for (const NavCelId c : o.cells) {
      one(c);
    }
  }
  return changed;
}

}  // namespace

std::ptrdiff_t combine_layer(
  NavMap & nm, const std::string & dst, const std::string & src, CombineOp op,
  const CombineOptions & options)
{
  const auto d = nm.layers.get(dst), s = nm.layers.get(src);
  const std::size_t n = nm.navcels.size();
  if (!d || !s || dst == src || d->size() != n || s->size() != n) {return -1;}
  std::shared_ptr<LayerView<uint8_t>> prov;
  if (!options.provenance.empty()) {
    if (options.provenance == dst || options.provenance == src) {return -1;}
    if (!nm.has_layer(options.provenance)) {
      nm.add_layer<uint8_t>(options.provenance, "source that set each cell (0: none)", "", 0);
    }
    prov = std::dynamic_pointer_cast<LayerView<uint8_t>>(nm.layers.get(options.provenance));
    if (!prov || prov->size() != n) {return -1;}
  }
  const auto meta = [&](const std::string & name) {
      const auto m = nm.get_layer_meta(name);
      return m ? m->unknown : std::nullopt;
    };
  const auto du = meta(dst), su = meta(src);
  std::ptrdiff_t out = -1;
  with_view(
    d, [&](auto & dv) {
      with_view(s, [&](auto & sv) {out = run(dv, sv, du, su, op, options, prov.get());});
    });
  return out;
}

}  // namespace navmap
