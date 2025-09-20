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


#ifndef NAVMAP_CORE__NAVMAP_HPP
#define NAVMAP_CORE__NAVMAP_HPP

/**
 * \file
 * \brief Core container and data structures for EasyNav navigable meshes.
 *
 * This header defines the fundamental types for NavMap:
 *  - Vertex storage as a structure-of-arrays (SoA) via ::navmap::Positions.
 *  - Triangle cells (::navmap::NavCel) with cached geometry and adjacency.
 *  - Surfaces (::navmap::Surface) as partitions of triangles with their own BVH.
 *  - A per-NavCel layer registry (::navmap::LayerRegistry) to store arbitrary
 *    scalar attributes (occupancy, cost, elevation, etc.).
 *  - The main ::navmap::NavMap container, including geometry queries such as
 *    raycast, closest-triangle and nav-cell localization.
 *
 * The API exposes low-level containers while providing convenience utilities
 * for common workflows without sacrificing control.
 */

#include <vector>
#include <memory>
#include <unordered_map>
#include <optional>
#include <string>
#include <array>
#include <limits>
#include <stack>
#include <cmath>
#include <cstdint>
#include <span>
#include <type_traits>
#include <deque>
#include <algorithm>

#include <Eigen/Core>
#include "navmap_core/Geometry.hpp"

namespace navmap
{

/// \brief Index into the per-vertex position arrays (SoA).
using PointId = uint32_t;
/// \brief Index of a triangle (NavCel) within the global mesh.
using NavCelId = uint32_t;

// -----------------------------------------------------------------------------
// Core data arrays
// -----------------------------------------------------------------------------

/**
 * \brief Structure-of-arrays for storing 3D vertex positions.
 *
 * Positions are stored in separate x/y/z arrays for cache-friendly access
 * and easy interop. Use ::navmap::Positions::at() to read a vertex as
 * an Eigen vector.
 *
 * \invariant size() == x.size() == y.size() == z.size()
 */
struct Positions
{
  std::vector<float> x;  ///< X coordinates (meters)
  std::vector<float> y;  ///< Y coordinates (meters)
  std::vector<float> z;  ///< Z coordinates (meters)

  /// \return Number of vertices stored.
  inline size_t size() const {return x.size();}

  /**
   * \brief Returns vertex \p id as a 3D vector.
   * \param id Vertex index (0-based).
   * \return Eigen::Vector3f with (x,y,z).
   * \warning No bounds checking is performed.
   */
  inline Eigen::Vector3f at(PointId id) const
  {
    return {x[id], y[id], z[id]};
  }
};

/**
 * \brief Optional per-vertex colors (RGBA, 8-bit per channel).
 *
 * When present, the length of each channel must match the number of vertices.
 */
struct Colors
{
  std::vector<uint8_t> r;  ///< Red   channel
  std::vector<uint8_t> g;  ///< Green channel
  std::vector<uint8_t> b;  ///< Blue  channel
  std::vector<uint8_t> a;  ///< Alpha channel
};

// -----------------------------------------------------------------------------
// Layers (dynamic per-NavCel attributes)
// -----------------------------------------------------------------------------

/**
 * \brief Runtime type tag for a layer's scalar storage.
 *
 * The registry currently supports: 8-bit unsigned, 32-bit float, 64-bit double.
 */
enum class LayerType : uint8_t { U8 = 0, F32 = 1, F64 = 2 };

/**
 * \brief Non-templated base for runtime layer handling.
 *
 * A layer stores one scalar value **per NavCel (triangle)**.
 */
struct LayerViewBase
{
  virtual ~LayerViewBase() = default;

  /// \return Type tag of the underlying storage.
  virtual LayerType type() const = 0;

  /// \return Layer name (unique within the registry).
  virtual const std::string & name() const = 0;

  /// \return Number of items (= number of NavCels).
  virtual size_t size() const = 0;

  /// @brief Return 64-bit content hash (cached; recomputed lazily).
  virtual std::uint64_t content_hash() const = 0;

  /// @brief Mark content dirty (forces hash recompute on next query).
  void mark_dirty() const noexcept {hash_dirty_ = true;}

protected:
  mutable bool hash_dirty_{true};
  mutable std::uint64_t hash_cache_{0};
};

/**
 * \brief Typed layer view storing one \p T value per NavCel.
 *
 * \tparam T Scalar type (uint8_t, float, double).
 * \note Elements are indexed by ::navmap::NavCelId.
 */
template<typename T>
struct LayerView : LayerViewBase
{
  std::string name_;     ///< Layer name (key in the registry)
  std::vector<T> data_;  ///< Values, one per NavCel
  LayerType type_;       ///< Runtime type tag (must match \p T)

  /**
   * \brief Construct a typed view.
   * \param name   Layer name.
   * \param nitems Number of NavCels.
   * \param t      Runtime type tag corresponding to \p T.
   */
  LayerView(std::string name, size_t nitems, LayerType t)
  : name_(std::move(name)), data_(nitems), type_(t) {}

  LayerType type() const override {return type_;}
  const std::string & name() const override {return name_;}
  size_t size() const override {return data_.size();}

  /// \name Element access (indexed by NavCelId).
  ///@{
  T & operator[](NavCelId cid) {return data_[cid];}
  const T & operator[](NavCelId cid) const {return data_[cid];}
  ///@}

  /// \return Mutable reference to internal storage.
  std::vector<T> & data() {return data_;}
  /// \return Const reference to internal storage.
  const std::vector<T> & data() const {return data_;}

  /// @name Layer hashing & access
  ///@{
  std::vector<T> & mutable_data() const
  {
    hash_dirty_ = true; return const_cast<std::vector<T> &>(data_);
  }
  void set_data(const std::vector<T> & v) {data_ = v; hash_dirty_ = true;}
  std::uint64_t content_hash() const override;
  ///@}
};

/**
 * \brief Registry of named layers (per-NavCel).
 *
 * Provides creation-or-lookup semantics with ::navmap::LayerRegistry::add_or_get().
 * All layers in the registry are expected to have a size equal to the number of
 * NavCels in the owning ::navmap::NavMap.
 */
class LayerRegistry {
public:
  /**
   * \brief Add a new typed layer or return an existing one with the same name.
   *
   * If a layer with \p name already exists, it is returned (no resize).
   * Otherwise a new layer is created with \p nitems elements.
   *
   * \tparam T Storage type (e.g., uint8_t, float, double).
   * \param name   Layer name (unique key).
   * \param nitems Number of NavCels to allocate.
   * \param type   Runtime type tag corresponding to \p T.
   * \return Shared pointer to the typed view.
   */
  template<typename T>
  std::shared_ptr<LayerView<T>> add_or_get(
    const std::string & name,
    size_t nitems,
    LayerType type)
  {
    auto it = layers_.find(name);
    if (it != layers_.end()) {
      return std::dynamic_pointer_cast<LayerView<T>>(it->second);
    }
    auto view = std::make_shared<LayerView<T>>(name, nitems, type);
    layers_[name] = view;
    return view;
  }

  /**
   * \brief Get an existing layer by name (untyped view).
   * \param name Layer name.
   * \return Pointer to base view, or nullptr if not found.
   */
  std::shared_ptr<LayerViewBase> get(const std::string & name) const
  {
    auto it = layers_.find(name);
    return it == layers_.end() ? nullptr : it->second;
  }

  /**
   * \brief List layer names currently in the registry.
   * \return Vector of names (unordered).
   */
  std::vector<std::string> list() const
  {
    std::vector<std::string> out;
    out.reserve(layers_.size());
    for (const auto & kv : layers_) {
      out.push_back(kv.first);
    }
    return out;
  }

  /**
   * \brief Remove a layer by name.
   * \return true if the layer existed and was removed.
   */
  bool remove(const std::string & name)
  {
    return layers_.erase(name) > 0;
  }

  /**
   * \brief Resize all known typed layers to \p nitems.
   *
   * Useful after changing the number of NavCels. Unknown types are ignored.
   *
   * \param nitems New number of items (NavCels).
   */
  void resize_all(size_t nitems)
  {
    for (auto & kv : layers_) {
      if (auto v = std::dynamic_pointer_cast<LayerView<uint8_t>>(kv.second)) {
        v->data_.resize(nitems);
        kv.second->mark_dirty();
        continue;
      }
      if (auto v = std::dynamic_pointer_cast<LayerView<float>>(kv.second)) {
        v->data_.resize(nitems);
        kv.second->mark_dirty();
        continue;
      }
      if (auto v = std::dynamic_pointer_cast<LayerView<double>>(kv.second)) {
        v->data_.resize(nitems);
        kv.second->mark_dirty();
        continue;
      }
    }
  }

private:
  std::unordered_map<std::string, std::shared_ptr<LayerViewBase>> layers_;
};

/**
 * \brief Metadata associated to a layer (optional).
 *
 * This metadata is kept alongside the runtime registry (out-of-band).
 */
struct LayerMeta
{
  std::string description;   ///< Human-readable description
  std::string unit;          ///< Physical unit (e.g., "m", "deg", "%")
  bool per_cell{true};       ///< true for per-NavCel layers (current behavior)
};

// -----------------------------------------------------------------------------
// Mesh cells and acceleration
// -----------------------------------------------------------------------------

/**
 * \brief Navigation cell (triangle) with geometry and adjacency.
 *
 * Stores three vertex indices, precomputed geometric data (normal, area),
 * and the indices of up to three neighboring NavCels across each edge.
 *
 * \note Layer values are stored per-NavCel in the ::navmap::LayerRegistry
 *       of the enclosing ::navmap::NavMap (not here).
 */
struct NavCel
{
  PointId v[3]{0, 0, 0};                      ///< Indices into ::navmap::Positions
  Eigen::Vector3f normal{0.0f, 0.0f, 1.0f};   ///< Unit normal of the triangle
  float area{0.0f};                            ///< Triangle area (m²)
  NavCelId neighbor[3]{                        ///< Neighbor cids across edges 0,1,2
    std::numeric_limits<uint32_t>::max(),
    std::numeric_limits<uint32_t>::max(),
    std::numeric_limits<uint32_t>::max()
  };
  uint32_t layer_dirty_mask{0};               ///< Reserved for future per-layer flags
};

/**
 * \brief Node in a per-surface bounding volume hierarchy (BVH).
 *
 * Leaves reference a compact range into the surface's primitive list.
 */
struct BVHNode
{
  AABB box;          ///< Bounding box of this node (world coordinates)
  int left{-1};      ///< Left child index (or -1 for leaf)
  int right{-1};     ///< Right child index (or -1 for leaf)
  int start{0};      ///< Start index in primitive array (leaf only)
  int count{0};      ///< Number of primitives in leaf (0 for inner nodes)
  bool is_leaf() const {return count > 0;}
};

/**
 * \brief A connected set of NavCels in a common reference frame.
 *
 * Each ::navmap::Surface owns a subset of NavCels plus its own BVH.
 * The \ref frame_id is provided for external consumers (e.g., ROS TF).
 */
struct Surface
{
  std::string frame_id;           ///< Frame id of this surface
  std::vector<NavCelId> navcels;  ///< NavCels belonging to this surface (global ids)
  AABB aabb;                      ///< Bounds of the surface geometry
  std::vector<int> prim_indices;  ///< Compact list of cids used by BVH leaves
  std::vector<BVHNode> bvh;       ///< BVH nodes for this surface
};

// -----------------------------------------------------------------------------
// Rays
// -----------------------------------------------------------------------------

/**
 * \brief Simple ray (origin + direction).
 *
 * \note Direction should be normalized for consistent \ref RayHit::t scaling.
 */
struct Ray
{
  Eigen::Vector3f o;  ///< Origin of the ray (world)
  Eigen::Vector3f d;  ///< Direction of the ray (unit length recommended)
};

/**
 * \brief Result of a raycast against the NavMap.
 *
 * All fields are valid only when \ref hit is true.
 */
struct RayHit
{
  bool hit{false};         ///< True if the ray hit any triangle
  size_t surface{0};       ///< Index of surface hit
  NavCelId cid{0};         ///< Id of the NavCel hit
  float t{0.0f};           ///< Distance along the ray (units of \ref Ray::d)
  Eigen::Vector3f p;       ///< World coordinates of intersection
};

// -----------------------------------------------------------------------------
// Helpers (type → LayerType mapping)
// -----------------------------------------------------------------------------

/// \brief Helper to map C++ scalar type to ::navmap::LayerType tag.
template<typename T> inline constexpr LayerType layer_type_tag();
template<> inline constexpr LayerType layer_type_tag<uint8_t>() {return LayerType::U8;}
template<> inline constexpr LayerType layer_type_tag<float>() {return LayerType::F32;}
template<> inline constexpr LayerType layer_type_tag<double>() {return LayerType::F64;}

/**
 * \brief Shape selector for area-writing APIs.
 */
enum class AreaShape { CIRCULAR, RECTANGULAR };

// -----------------------------------------------------------------------------
// NavMap: public API
// -----------------------------------------------------------------------------

/**
 * \brief Main container for navigable surfaces, geometry, and layers.
 *
 * A ::navmap::NavMap aggregates vertex positions, a list of ::navmap::NavCel
 * triangles (global indexing), one or more ::navmap::Surface partitions, and
 * a ::navmap::LayerRegistry with arbitrary per-NavCel scalar attributes.
 *
 * **Typical workflow**
 *  - Fill \ref positions, \ref navcels, and \ref surfaces.
 *  - Call \ref rebuild_geometry_accels() to compute normals, areas, adjacency,
 *    and build per-surface BVHs.
 *  - Create layers via \ref LayerRegistry::add_or_get() sized to navcels.size().
 *  - Query with \ref locate_navcel(), \ref raycast(), or \ref closest_triangle().
 */
class NavMap {
public:
  Positions positions;                 ///< Vertex positions (SoA)
  std::optional<Colors> colors;        ///< Optional per-vertex colors
  std::vector<NavCel> navcels;         ///< All triangles (global indexing)
  std::vector<Surface> surfaces;       ///< Surfaces (partitions of \ref navcels)
  LayerRegistry layers;                ///< Per-NavCel layers (runtime registry)
  std::unordered_map<std::string, LayerMeta> layer_meta;  ///< Optional metadata per layer

  /// \brief Copy constructor: deep copy of geometry, surfaces, layers and metadata.
  NavMap();

  /// \brief Copy constructor: deep copy of geometry, surfaces, layers and metadata.
  NavMap(const NavMap & other);

  /**
   * \brief Recompute derived geometry and acceleration structures.
   *
   * Computes triangle normals and areas, builds adjacency between neighbors,
   * and builds per-surface BVHs. Layer sizes are not changed automatically;
   * call \ref LayerRegistry::resize_all() if you modified \ref navcels.
   */
  void rebuild_geometry_accels();

/**
 * \brief Copy assignment optimized to avoid geometry duplication.
 *
 * If both maps share identical geometry (same vertex arrays and NavCel indices),
 * layers are synchronized by name/type (copy-on-difference) and destination-only layers are removed.
 * Otherwise, a full deep copy is performed.
 */
  NavMap & operator=(const NavMap & other);

/**
 * \brief Move assignment.
 * Transfers ownership of geometry, surfaces, layers and metadata.
 */
  NavMap & operator=(NavMap && other) noexcept;

/// \brief Fast check for identical geometry (vertices and NavCel indices).
  bool has_same_geometry(const NavMap & other) const;

  /**
   * \brief Build topological adjacency between neighboring NavCels.
   *
   * Two triangles are neighbors if they share an undirected edge.
   * Called by \ref rebuild_geometry_accels().
   */
  void build_adjacency();

  /**
   * \brief Mark a vertex as updated (reserved for future cache invalidation).
   * \param pid Vertex id (unused for now).
   */
  void mark_vertex_updated(PointId /*pid*/) {}

  /**
   * \brief Return the three neighbor NavCel ids of triangle \p cid.
   * \param cid NavCel id.
   * \return Array with neighbor ids or max uint32_t if boundary.
   */
  std::array<NavCelId, 3> get_neighbors(NavCelId cid) const
  {
    return {navcels[cid].neighbor[0],
      navcels[cid].neighbor[1],
      navcels[cid].neighbor[2]};
  }

  /**
   * \brief Raycast against all surfaces to find the closest hit.
   * \param o Ray origin (world).
   * \param d Ray direction (normalized).
   * \param[out] hit_cid NavCel id hit (valid if return is true).
   * \param[out] t Distance along ray (valid if return is true).
   * \param[out] hit_pt World-space intersection point.
   * \return true if any triangle was hit.
   */
  bool raycast(
    const Eigen::Vector3f & o,
    const Eigen::Vector3f & d,
    NavCelId & hit_cid,
    float & t,
    Eigen::Vector3f & hit_pt) const;

  /**
   * \brief Batched raycast.
   * \param rays Input rays.
   * \param[out] out Output hits (parallel to \p rays).
   * \param first_hit_only If true, stop at the first surface that hits.
   */
  void raycast_many(
    const std::vector<Ray> & rays,
    std::vector<RayHit> & out,
    bool first_hit_only = true) const;

  // --- Per-NavCel layer value access ---

  /**
   * \brief Read the value of a typed per-NavCel layer at triangle \p cid.
   * \tparam T Layer storage type (must match the layer).
   * \param cid   NavCel id.
   * \param layer Typed layer view.
   * \return The value for triangle \p cid.
   */
  template<typename T>
  T navcel_value(NavCelId cid, const LayerView<T> & layer) const
  {
    return layer[cid];
  }

  /**
   * \brief Options for the locate functions.
   *
   * - \ref hint_cid : starting triangle for walking (if provided).
   * - \ref hint_surface : optional surface restriction.
   * - \ref planar_eps : in-triangle barycentric tolerance.
   * - \ref height_eps : vertical tolerance when gating by AABB / fallback.
   * - \ref use_downward_ray : select downward or upward vertical ray fallback.
   */
  struct LocateOpts
  {
    std::optional<NavCelId> hint_cid;    ///< Optional triangle hint for walking
    std::optional<size_t> hint_surface;  ///< Optional surface hint
    float planar_eps = 1e-4f;            ///< In-plane barycentric tolerance
    float height_eps = 0.50f;            ///< Z tolerance for vertical fallback (meters)
    bool use_downward_ray = true;        ///< Downward ray on fallback (else upward)
  };

  /**
   * \brief Locate the triangle under / near a world point (convenience).
   *
   * Uses default \ref LocateOpts. See the full overload for details.
   *
   * \param p_world     Query point in world coordinates.
   * \param[out] surface_idx Surface index owning the located triangle.
   * \param[out] cid    Located NavCel id.
   * \param[out] bary   Barycentric coordinates of the hit.
   * \param[out] hit_pt Optional: projected point on the surface.
   * \return true if a triangle has been located.
   */
  bool locate_navcel(
    const Eigen::Vector3f & p_world,
    size_t & surface_idx,
    NavCelId & cid,
    Eigen::Vector3f & bary,
    Eigen::Vector3f * hit_pt) const;

  /**
   * \brief Locate the triangle under / near a world point.
   *
   * Strategy:
   *  1) If \ref LocateOpts::hint_cid is provided, try walking neighbors.
   *  2) Else, try a per-surface 2D seed grid near (x,y) with planar test.
   *  3) If still not found, vertical raycast (downward or upward).
   *
   * \param p_world     Query point in world coordinates.
   * \param[out] surface_idx Surface index owning the located triangle.
   * \param[out] cid    Located NavCel id.
   * \param[out] bary   Barycentric coordinates of the hit.
   * \param[out] hit_pt Optional: projected point on the surface.
   * \param opts        Tuning options (see \ref LocateOpts).
   * \return true if a triangle has been located.
   */
  bool locate_navcel(
    const Eigen::Vector3f & p_world,
    size_t & surface_idx,
    NavCelId & cid,
    Eigen::Vector3f & bary,
    Eigen::Vector3f * hit_pt,
    const LocateOpts & opts) const;

  /**
   * \brief Find the closest triangle to a point.
   *
   * Traverses per-surface BVHs with distance lower-bounds; returns the
   * closest triangle, the closest point on it, and the squared distance.
   *
   * \param p_world      Query point in world coordinates.
   * \param[out] surface_idx Surface index of the closest triangle.
   * \param[out] cid     NavCel id of the closest triangle.
   * \param[out] closest_point Closest point on that triangle.
   * \param[out] sqdist  Squared distance to \p p_world.
   * \param restrict_surface If >= 0, restrict search to this surface.
   * \return true if any triangle was considered.
   */
  bool closest_triangle(
    const Eigen::Vector3f & p_world,
    size_t & surface_idx,
    NavCelId & cid,
    Eigen::Vector3f & closest_point,
    float & sqdist,
    int restrict_surface = -1) const;

  // ---- Convenience: construction & editing (additive, backward-compatible) ----

  /// \brief Create a new empty Surface and append it to the map.
  /// \param frame_id TF frame id for the surface.
  /// \return Index of the created surface in \ref surfaces.
  std::size_t create_surface(std::string frame_id);

  /// \brief Create a standalone Surface object (not yet added to the map).
  /// \param frame_id Frame id to assign.
  /// \return A Surface with no navcels, ready to be customized and then added.
  Surface create_surface_obj(const std::string & frame_id) const;

  /// \brief Append an existing Surface (by value). \return New index in \ref surfaces.
  std::size_t add_surface(const Surface & s);

  /// \brief Add a Surface by move (avoids copy). \return New index in \ref surfaces.
  std::size_t add_surface(Surface && s);

  /// \brief Remove a Surface by index. Does NOT touch \ref navcels / \ref positions.
  /// \note The caller must ensure there are no dangling indices.
  /// \return true if the surface existed and was removed.
  bool remove_surface(std::size_t surface_index);

  /// \brief Append a vertex and return its index.
  uint32_t add_vertex(const Eigen::Vector3f & p);

  /// \brief Append a triangle (NavCel) and return its id.
  /// \param v0 Index of vertex 0.
  /// \param v1 Index of vertex 1.
  /// \param v2 Index of vertex 2.
  NavCelId add_navcel(uint32_t v0, uint32_t v1, uint32_t v2);

  /// \brief Add an existing nav cell id to a surface.
  void add_navcel_to_surface(std::size_t surface_index, NavCelId cid);

  /// \brief Triangle centroid (computed on the fly).
  Eigen::Vector3f navcel_centroid(NavCelId cid) const;

  /// \brief Return up to 3 neighbor cell ids (skips invalid entries).
  std::vector<NavCelId> navcel_neighbors(NavCelId cid) const;

  // ---- Layers (per-NavCel) convenience ----

  /**
   * \brief Create (or get) a typed per-NavCel layer with a default value.
   *
   * If a layer with \p name exists, it is returned (size preserved). If the
   * size differs from \ref navcels.size(), values are reinitialized to
   * \p default_value.
   *
   * \tparam T Storage type.
   * \param name          Layer name.
   * \param description   Human-readable description (optional).
   * \param unit          Unit (e.g., "m", "deg", "%"), optional.
   * \param default_value Initial value for all triangles.
   * \return Shared view of the created/updated layer.
   */
  template<typename T>
  std::shared_ptr<LayerView<T>> add_layer(
    const std::string & name,
    const std::string & description = {},
    const std::string & unit = {},
    T default_value = T{})
  {
    auto v = layers.add_or_get<T>(name, navcels.size(), layer_type_tag<T>());
    if (v->data().size() != navcels.size()) {
      v->data().assign(navcels.size(), default_value);
    }
    layer_meta[name] = LayerMeta{description, unit, true};
    return v;
  }

  /// \brief Check whether a layer named \p name exists.
  bool has_layer(const std::string & name) const;

  /// \brief Number of entries in a layer (should equal navcels.size()).
  std::size_t layer_size(const std::string & name) const;

  /// \brief Human-readable type name for a layer ("float", "double", "uint8", ...).
  std::string layer_type_name(const std::string & name) const;

  /**
   * \brief Set a per-cell value (creates/retypes the layer if needed).
   * \tparam T Storage type.
   * \param name  Layer name.
   * \param cid   Triangle id.
   * \param value Value to write.
   */
  template<typename T>
  void layer_set(const std::string & name, NavCelId cid, T value)
  {
    auto view = layers.add_or_get<T>(name, navcels.size(), layer_type_tag<T>());
    if (static_cast<size_t>(cid) < view->data().size()) {
      view->data()[cid] = value;
      view->mark_dirty();
    }
  }

  /**
   * \brief Get a per-cell value as the requested type.
   *
   * If the stored type matches \p T, returns directly. Otherwise falls back to
   * conversion vía double: floating → static_cast<T>, integral → clamp [0,max(T)]
   * y redondeo (llround).
   *
   * Si la capa no existe o \p cid está fuera de rango, devuelve \p def.
   *
   * \tparam T uint8_t, float, or double
   * \param name Layer name
   * \param cid  Triangle id
   * \param def  Default value on failure
   * \return Value converted to T, or \p def
   */
  template<typename T>
  T layer_get(const std::string & name, NavCelId cid, T def = T{}) const
  {
    if (auto base = layers.get(name)) {
      if (auto view = std::dynamic_pointer_cast<LayerView<T>>(base)) {
        const auto & buf = view->data();
        if (static_cast<size_t>(cid) < buf.size()) {
          return buf[cid];
        }
        return def;
      }
    }
    // Fallback: try to fetch as double from any stored type and convert
    double v;
    if (auto base = layers.get(name)) {
      if (auto v8 = std::dynamic_pointer_cast<LayerView<uint8_t>>(base)) {
        const auto & buf = v8->data();
        if (static_cast<size_t>(cid) >= buf.size()) {return def;}
        v = static_cast<double>(buf[cid]);
      } else if (auto vf = std::dynamic_pointer_cast<LayerView<float>>(base)) {
        const auto & buf = vf->data();
        if (static_cast<size_t>(cid) >= buf.size()) {return def;}
        v = static_cast<double>(buf[cid]);
      } else if (auto vd = std::dynamic_pointer_cast<LayerView<double>>(base)) {
        const auto & buf = vd->data();
        if (static_cast<size_t>(cid) >= buf.size()) {return def;}
        v = buf[cid];
      } else {
        return def;
      }
    } else {
      return def;
    }

    if constexpr (std::is_floating_point_v<T>) {
      return static_cast<T>(v);
    } else if constexpr (std::is_integral_v<T>) {
      double lo = 0.0;
      double hi = static_cast<double>(std::numeric_limits<T>::max());
      double vv = v < lo ? lo : (v > hi ? hi : v);
      return static_cast<T>(std::llround(vv));
    } else {
      static_assert(sizeof(T) == 0, "Unsupported layer type");
    }
  }

  /**
   * \brief Return metadata for a layer if present (implementation-defined).
   * \param name Layer name.
   * \return Optional metadata if available.
   */
  std::optional<LayerMeta> get_layer_meta(const std::string & name) const;

  /**
   * \brief List all layer names currently stored.
   * \return Vector of layer names.
   */
  std::vector<std::string> list_layers() const;

  /**
   * \brief Sample a per-NavCel layer at a world position.
   *
   * Locates the containing triangle and returns its layer value.
   * \param name    Layer name.
   * \param p_world World-space query.
   * \param def     Default value if not found.
   * \return Value or \p def if missing.
   */
  double sample_layer_at(
    const std::string & name,
    const Eigen::Vector3f & p_world,
    double def = std::numeric_limits<double>::quiet_NaN()) const;

  /**
   * \brief Reset all values in a layer to a given value.
   *
   * \param T Expected storage type of the layer.
   * \param name Layer name.
   * \param value Value to assign to all entries.
   */
  template<typename T>
  void layer_clear(const std::string & name, T value = T{})
  {
    auto view = layers.add_or_get<T>(name, navcels.size(), layer_type_tag<T>());
    std::fill(view->data().begin(), view->data().end(), value);
    view->mark_dirty();
  }

  /**
   * \brief Copy values from one layer into another.
   *
   * Both layers must have the same type and size.
   *
   * \param T Storage type (e.g. float, uint8_t).
   * \param src Name of source layer.
   * \param dst Name of destination layer.
   * \return true if copy succeeded, false if type/size mismatch.
   */
  template<typename T>
  bool layer_copy(const std::string & src, const std::string & dst)
  {
    // No-op if names are equal
    if (src == dst) {
      return true;
    }

    // Get source
    auto src_base = layers.get(src);
    auto src_view = std::dynamic_pointer_cast<LayerView<T>>(src_base);
    if (!src_view) {
      return false; // source layer missing or wrong type
    }

    // Ensure destination exists with correct length (navcels.size())
    auto dst_view = layers.add_or_get<T>(dst, navcels.size(), layer_type_tag<T>());
    if (!dst_view) {
      return false;
    }

    // If sizes mismatch, something is wrong with geometry/layer size.
    // You can choose to resize here if your semantics allow it. For safety, fail:
    if (src_view->data().size() != dst_view->data().size()) {
      return false;
    }

    // If both views already refer to the same underlying buffer, no work to do
    if (!src_view->data().empty() &&
      src_view->data().data() == dst_view->data().data())
    {
      return true;
    }

    // Hash-based skip: O(1) if cached, O(n) only on first compute (then cached)
    const bool same_hash = (src_view->content_hash() == dst_view->content_hash());
    if (same_hash) {
      return true; // identical content → avoid copy
    }

    // Copy only when different. set_data() marks hash as dirty for dst.
    dst_view->set_data(src_view->data());
    return true;
  }

  /**
   * \brief Create (or get) a typed per-NavCel layer, initialized from another layer.
   *
   * If a layer with \p name exists, it is returned (size preserved). If the
   * size differs from \ref navcels.size(), it is resized. If the source layer
   * \p src_layer exists, its values are copied into the destination using
   * \ref layer_copy(), performing type conversion if necessary.
   * If the source layer does not exist, the destination is left with defaults.
   *
   * \tparam T Storage type.
   * \param name        Layer name.
   * \param description Human-readable description (optional).
   * \param unit        Unit (e.g., "m", "deg", "%"), optional.
   * \param src_layer   Source layer name to initialize from.
   * \return Shared view of the created/updated layer.
   */
  template<typename T>
  std::shared_ptr<LayerView<T>> add_layer(
    const std::string & name,
    const std::string & description,
    const std::string & unit,
    const std::string & src_layer)
  {
    auto dst = layers.add_or_get<T>(name, navcels.size(), layer_type_tag<T>());
    if (dst->data().size() != navcels.size()) {
      dst->data().assign(navcels.size(), T{});
    }
    layer_meta[name] = LayerMeta{description, unit, true};

    if (!has_layer(src_layer)) {
      return dst;
    }

    if (layer_copy<T>(src_layer, name)) {
      return dst;
    }

    const std::size_t n = dst->size();
    for (NavCelId cid = 0; cid < n; ++cid) {
      double v = layer_get<double>(src_layer, cid, std::numeric_limits<double>::quiet_NaN());
      if (std::isnan(v)) {v = 0.0;}

      if constexpr (std::is_floating_point_v<T>) {
        dst->data()[cid] = static_cast<T>(v);
      } else if constexpr (std::is_integral_v<T>) {
        double lo = 0.0;
        double hi = static_cast<double>(std::numeric_limits<T>::max());
        if (v < lo) {v = lo;}
        if (v > hi) {v = hi;}
        dst->data()[cid] = static_cast<T>(std::llround(v));
      } else {
        static_assert(sizeof(T) == 0, "Unsupported layer type");
      }
    }

    return dst;
  }

  /**
   * \brief Forward to the existing low-level locator (implemented in .cpp).
   *
   * This hook should call your original, detailed locator implementation.
   * The robust overload above will fall back to vertical rays if needed.
   */
  bool locate_navcel_core(
    const Eigen::Vector3f & p_world,
    std::size_t & surface_idx,
    NavCelId & cid,
    Eigen::Vector3f & bary,
    Eigen::Vector3f * hit_pt,
    const LocateOpts & opts) const;

  /**
   * \brief Set a per-NavCel layer to a constant value over a 2D area.
   *
   * The center is the ground projection of \p p_world. If the projected NavCel
   * cannot be determined, returns false. Supports U8/F32/F64 layers.
   *
   * Efficiency: BFS from the seed NavCel with XY AABB pruning against the
   * circumscribed square of the area. Inclusion test by triangle centroid.
   *
   * \tparam T uint8_t, float, or double
   * \param p_world 3D point in world coordinates (projected to ground)
   * \param value Value to set in the layer
   * \param layer_name Target layer name
   * \param shape CIRCULAR or RECTANGULAR
   * \param size Radius (CIRCULAR) or side length (RECTANGULAR)
   * \return true if the seed NavCel was located and the layer updated
   */
  template<typename T>
  bool set_area(
    const Eigen::Vector3f & p_world,
    T value,
    const std::string & layer_name,
    AreaShape shape,
    float size)
  {
    if (navcels.empty() || surfaces.empty() || size <= 0.0f) {
      return false;
    }

    std::size_t surface_idx = 0;
    NavCelId start_cid{};
    Eigen::Vector3f bary{};
    Eigen::Vector3f hit_pt{};
    if (!locate_navcel(p_world, surface_idx, start_cid, bary, &hit_pt)) {
      return false;
    }

    const float cx = hit_pt.x();
    const float cy = hit_pt.y();

    {
      auto existing = layers.get(layer_name);
      if (existing && existing->type() != layer_type_tag<T>()) {
        return false;  // type mismatch
      }
    }
    auto layer = layers.add_or_get<T>(layer_name, navcels.size(), layer_type_tag<T>());
    if (!layer) {
      return false;
    }
    if (layer->data().size() != navcels.size()) {
      const_cast<std::vector<T> &>(layer->data()).resize(navcels.size(), T{});
    }

    const float half = (shape == AreaShape::CIRCULAR) ? size : (0.5f * size);
    const float minx = cx - half, maxx = cx + half;
    const float miny = cy - half, maxy = cy + half;
    const float r2 = (shape == AreaShape::CIRCULAR) ? (size * size) : 0.0f;

    auto tri_aabb_intersects_box = [&](NavCelId cid) -> bool {
        const auto & tri = navcels[static_cast<std::size_t>(cid)];
        const Eigen::Vector3f p0{positions.x[tri.v[0]], positions.y[tri.v[0]],
          positions.z[tri.v[0]]};
        const Eigen::Vector3f p1{positions.x[tri.v[1]], positions.y[tri.v[1]],
          positions.z[tri.v[1]]};
        const Eigen::Vector3f p2{positions.x[tri.v[2]], positions.y[tri.v[2]],
          positions.z[tri.v[2]]};
        const float tminx = std::min({p0.x(), p1.x(), p2.x()});
        const float tmaxx = std::max({p0.x(), p1.x(), p2.x()});
        const float tminy = std::min({p0.y(), p1.y(), p2.y()});
        const float tmaxy = std::max({p0.y(), p1.y(), p2.y()});
        if (tmaxx < minx || tminx > maxx || tmaxy < miny || tminy > maxy) {
          return false;
        }
        return true;
      };

    auto inside_area = [&](const Eigen::Vector3f & c) -> bool {
        const float dx = c.x() - cx;
        const float dy = c.y() - cy;
        if (shape == AreaShape::CIRCULAR) {
          return (dx * dx + dy * dy) <= r2;
        } else {
          return (std::abs(dx) <= half) && (std::abs(dy) <= half);
        }
      };

    std::vector<char> visited(navcels.size(), 0);
    std::deque<NavCelId> q;
    q.push_back(start_cid);
    visited[static_cast<std::size_t>(start_cid)] = 1;

    auto & data = layer->mutable_data();

    while (!q.empty()) {
      const NavCelId cid = q.front(); q.pop_front();

      if (!tri_aabb_intersects_box(cid)) {
        continue;
      }

      const Eigen::Vector3f c = navcel_centroid(cid);
      if (inside_area(c)) {
        data[static_cast<std::size_t>(cid)] = value;
      }

      for (const auto ncid : navcel_neighbors(cid)) {
        const std::size_t nidx = static_cast<std::size_t>(ncid);
        if (nidx < navcels.size() && !visited[nidx] && tri_aabb_intersects_box(ncid)) {
          visited[nidx] = 1;
          q.push_back(ncid);
        }
      }
    }

    return true;
  }

private:
  // Geometry fingerprint cache (lazy recompute)
  void ensure_geometry_fingerprint_() const;
  static std::uint64_t hash_geometry_bytes_(
    const float *x, std::size_t nx,
    const float *y, std::size_t ny,
    const float *z, std::size_t nz,
    const std::uint32_t *v0, std::size_t nv0,
    const std::uint32_t *v1, std::size_t nv1,
    const std::uint32_t *v2, std::size_t nv2);

  mutable bool geometry_dirty_{true};
  mutable std::uint64_t geometry_fp_{0};

  // Builders and traversal helpers.

  /**
   * \brief Build a per-surface BVH for fast ray queries.
   * \param s Surface to process (in-place).
   */
  void build_surface_bvh(Surface & s);

  /**
   * \brief Raycast against a single surface BVH.
   * \param s       Surface to test.
   * \param o       Ray origin (world).
   * \param d       Ray direction (normalized).
   * \param[out] hit_cid  Triangle id on hit.
   * \param[out] t_out    Distance along ray.
   * \param[out] hit_pt   Intersection point (world).
   * \return true if any triangle was hit.
   */
  bool surface_raycast(
    const Surface & s,
    const Eigen::Vector3f & o,
    const Eigen::Vector3f & d,
    NavCelId & hit_cid,
    float & t_out,
    Eigen::Vector3f & hit_pt) const;

  /**
   * \brief Walk across neighbors starting from \p start_cid to locate \p p.
   * \param start_cid  Starting triangle id.
   * \param p          Query point (world).
   * \param[out] cid_out Located triangle id.
   * \param[out] bary     Barycentric coordinates of the hit.
   * \param[out] hit_pt   Optional projected point on the surface.
   * \param planar_eps    In-plane barycentric tolerance.
   * \return true if \p p projects barycentrically inside some triangle.
   */
  bool locate_by_walking(
    NavCelId start_cid,
    const Eigen::Vector3f & p,
    NavCelId & cid_out,
    Eigen::Vector3f & bary,
    Eigen::Vector3f * hit_pt,
    float planar_eps) const;

  /**
   * \brief BVH traversal to find the closest triangle in one surface.
   * \param s   Surface to query.
   * \param p   World point.
   * \param[out] cid       Closest triangle id.
   * \param[out] q         Closest point on that triangle.
   * \param[in,out] best_sq On input: current best squared distance;
   *                        on output: improved best squared distance.
   * \return true if any candidate improved the best squared distance.
   */
  bool surface_closest_triangle(
    const Surface & s,
    const Eigen::Vector3f & p,
    NavCelId & cid,
    Eigen::Vector3f & q,
    float & best_sq) const;
};

// Inline convenience overload.
inline bool NavMap::locate_navcel(
  const Eigen::Vector3f & p_world,
  size_t & surface_idx,
  NavCelId & cid,
  Eigen::Vector3f & bary,
  Eigen::Vector3f * hit_pt) const
{
  return locate_navcel(p_world, surface_idx, cid, bary, hit_pt, LocateOpts{});
}

}  // namespace navmap

#endif  // NAVMAP_CORE__NAVMAP_HPP
