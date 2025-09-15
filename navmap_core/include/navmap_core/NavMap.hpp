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


#ifndef NAVMAP_CORE__NAVMAP_HPP
#define NAVMAP_CORE__NAVMAP_HPP

#include <vector>
#include <memory>
#include <unordered_map>
#include <optional>
#include <string>
#include <array>
#include <limits>
#include <stack>
#include <cmath>

#include <Eigen/Core>
#include "navmap_core/Geometry.hpp"

namespace navmap
{

using PointId = uint32_t;   ///< Index into position arrays (per-vertex).
using NavCelId = uint32_t;  ///< Index of a triangle (NavCel) in the mesh.

// -----------------------------------------------------------------------------
// Core data arrays
// -----------------------------------------------------------------------------

/**
 * @brief Structure-of-arrays for storing 3D vertex positions.
 *
 * Positions are stored in separate x/y/z arrays for cache-friendly access
 * and easy interop. Use @ref at() to read a vertex as Eigen::Vector3f.
 *
 * @invariant size() == x.size() == y.size() == z.size()
 */
struct Positions
{
  std::vector<float> x;  ///< X coordinates
  std::vector<float> y;  ///< Y coordinates
  std::vector<float> z;  ///< Z coordinates

  /// @return number of vertices.
  inline size_t size() const {return x.size();}

  /**
   * @brief Returns vertex @p id as a 3D vector.
   * @param id Vertex index.
   * @return Eigen::Vector3f with (x,y,z).
   * @warning No bounds checking is performed.
   */
  inline Eigen::Vector3f at(PointId id) const
  {
    return {x[id], y[id], z[id]};
  }
};

/**
 * @brief Optional per-vertex colors (RGBA, 8-bit per channel).
 *
 * When present, the length of each channel must match the number of vertices.
 */
struct Colors
{
  std::vector<uint8_t> r;  ///< Red
  std::vector<uint8_t> g;  ///< Green
  std::vector<uint8_t> b;  ///< Blue
  std::vector<uint8_t> a;  ///< Alpha
};

// -----------------------------------------------------------------------------
// Layers (dynamic per-NavCel attributes)
// -----------------------------------------------------------------------------

/**
 * @brief Runtime type tag for a layer's scalar storage.
 */
enum class LayerType : uint8_t { U8 = 0, F32 = 1, F64 = 2 };

/**
 * @brief Non-templated base for runtime layer handling.
 *
 * A layer stores one scalar value **per NavCel (triangle)**.
 */
struct LayerViewBase
{
  virtual ~LayerViewBase() = default;

  /// @return type tag of the underlying storage.
  virtual LayerType type() const = 0;

  /// @return layer name (unique within the registry).
  virtual const std::string & name() const = 0;

  /// @return number of items (= number of NavCels).
  virtual size_t size() const = 0;  // number of items (== number of NavCels)
};

/**
 * @brief Typed layer view storing one @p T value per NavCel.
 *
 * @tparam T Scalar type (uint8_t, float, double).
 * @note Elements are indexed by @ref NavCelId.
 */
template<typename T>
struct LayerView : LayerViewBase
{
  std::string name_;     ///< Layer name.
  std::vector<T> data_;  ///< Values, one per NavCel.
  LayerType type_;       ///< Runtime type tag.

  /**
   * @brief Construct a typed view.
   * @param name Layer name.
   * @param nitems Number of NavCels.
   * @param t Runtime type tag (must match T).
   */
  LayerView(std::string name, size_t nitems, LayerType t)
  : name_(std::move(name)), data_(nitems), type_(t) {}

  LayerType type() const override {return type_;}
  const std::string & name() const override {return name_;}
  size_t size() const override {return data_.size();}

  /// @name Element access (indexed by NavCelId).
  ///@{
  T & operator[](NavCelId cid) {return data_[cid];}
  const T & operator[](NavCelId cid) const {return data_[cid];}
  ///@}

  /// @return mutable reference to internal storage.
  std::vector<T> & data() {return data_;}
  /// @return const reference to internal storage.
  const std::vector<T> & data() const {return data_;}
};

/**
 * @brief Registry of named layers (per-NavCel).
 *
 * Provides creation-or-lookup semantics with @ref add_or_get().
 * All layers in the registry are expected to have a size equal
 * to the number of NavCels in the owning @ref NavMap.
 */
class LayerRegistry {
public:
  /**
   * @brief Add a new typed layer or return an existing one with the same name.
   *
   * If a layer with @p name already exists, it is returned (no resize).
   * Otherwise a new layer is created with @p nitems elements.
   *
   * @tparam T Storage type (e.g., uint8_t, float, double).
   * @param name Layer name (unique key).
   * @param nitems Number of NavCels to allocate.
   * @param type Runtime type tag corresponding to T.
   * @return Shared pointer to the typed view.
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
   * @brief Get an existing layer by name (untyped view).
   * @param name Layer name.
   * @return Pointer to base view, or nullptr if not found.
   */
  std::shared_ptr<LayerViewBase> get(const std::string & name) const
  {
    auto it = layers_.find(name);
    return it == layers_.end() ? nullptr : it->second;
  }

  /**
   * @brief List layer names currently in the registry.
   * @return Vector of names.
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
   * @brief Resize all known typed layers to @p nitems.
   *
   * Useful after changing the number of NavCels.
   * Unknown types are ignored.
   *
   * @param nitems New number of items (NavCels).
   */
  void resize_all(size_t nitems)
  {
    for (auto & kv : layers_) {
      // Only resize if underlying vector exists; keep type/name
      // Dynamic cast for known concrete type buckets
      // U8
      if (auto v = std::dynamic_pointer_cast<LayerView<uint8_t>>(kv.second)) {
        v->data_.resize(nitems);
        continue;
      }
      // F32
      if (auto v = std::dynamic_pointer_cast<LayerView<float>>(kv.second)) {
        v->data_.resize(nitems);
        continue;
      }
      // F64
      if (auto v = std::dynamic_pointer_cast<LayerView<double>>(kv.second)) {
        v->data_.resize(nitems);
        continue;
      }
    }
  }

private:
  std::unordered_map<std::string, std::shared_ptr<LayerViewBase>> layers_;
};

// -----------------------------------------------------------------------------
// Mesh cells and acceleration
// -----------------------------------------------------------------------------

/**
 * @brief Navigation cell (triangle) with geometry and adjacency.
 *
 * Stores three vertex indices, precomputed geometric data (normal, area),
 * and the indices of up to three neighboring NavCels across each edge.
 *
 * @note Layer values are no longer stored per-vertex, but per-NavCel and
 *       live in the @ref LayerRegistry of the enclosing @ref NavMap.
 */
struct NavCel
{
  PointId v[3]{0, 0, 0};          ///< Indices into @ref Positions
  Eigen::Vector3f normal{0.0f, 0.0f, 1.0f};    ///< Unit normal
  float area{0.0f};               ///< Triangle area
  NavCelId neighbor[3]{           ///< Neighbor cids across edges 0,1,2
    std::numeric_limits<uint32_t>::max(),
    std::numeric_limits<uint32_t>::max(),
    std::numeric_limits<uint32_t>::max()
  };
  uint32_t layer_dirty_mask{0};   ///< Reserved for future per-layer flags
};

/**
 * @brief Node in a per-surface bounding volume hierarchy (BVH).
 *
 * Leaves reference a compact range into the surface's primitive list.
 */
struct BVHNode
{
  AABB box;     ///< Bounding box of this node
  int left{-1};     ///< Left child index (or -1)
  int right{-1};    ///< Right child index (or -1)
  int start{0};     ///< Start index in primitive array (leaf)
  int count{0};     ///< Number of primitives in leaf (0 for inner nodes)
  bool is_leaf() const {return count > 0;}
};

/**
 * @brief Lightweight 2D uniform grid for accelerating point location.
 *
 * Used as a seed structure for @ref locate_navcel() when no hint is provided.
 * Buckets store NavCel indices whose XY AABB overlaps the cell.
 */
struct UniformGrid2D
{
  Eigen::Vector2f origin{0.0f, 0.0f};     ///< Grid origin (XY)
  Eigen::Vector2f cell_size{0.25f, 0.25f};   ///< Size of each cell
  int nx{0};    ///< Number of cells in X
  int ny{0};    ///< Number of cells in Y
  std::vector<std::vector<int>> buckets;  ///< Cell -> list of candidate cids

  /// @return true if the grid is initialized and non-empty.
  inline bool valid() const
  {
    return nx > 0 && ny > 0 && !buckets.empty();
  }

  /**
   * @brief Convert cell coordinates to linear index.
   * @param ix Cell x.
   * @param iy Cell y.
   * @return Linear index or -1 if out of bounds.
   */
  inline int index(int ix, int iy) const
  {
    if (ix < 0 || iy < 0 || ix >= nx || iy >= ny) {
      return -1;
    }
    return iy * nx + ix;
  }

  /**
   * @brief Return the cell containing point @p p (XY-plane).
   * @param p XY point.
   * @return Integer cell coordinates (may be outside [0,nx/ny)).
   */
  inline Eigen::Vector2i cell_of(const Eigen::Vector2f & p) const
  {
    int ix = static_cast<int>(std::floor((p.x() - origin.x()) / cell_size.x()));
    int iy = static_cast<int>(std::floor((p.y() - origin.y()) / cell_size.y()));
    return {ix, iy};
  }
};

/**
 * @brief A connected set of NavCels in a common reference frame.
 *
 * Each @ref Surface owns a subset of NavCels, plus its own BVH and
 * uniform seed grid. The @ref frame_id is for external consumers (ROS).
 */
struct Surface
{
  std::string frame_id;           ///< Frame id of this surface
  std::vector<NavCelId> navcels;  ///< NavCels belonging to this surface
  AABB aabb;                      ///< Bounds of the surface geometry
  std::vector<int> prim_indices;  ///< Compact list of cids used by BVH leaves
  std::vector<BVHNode> bvh;       ///< BVH nodes
  UniformGrid2D grid;             ///< Seed grid for locate()
};

// -----------------------------------------------------------------------------
// Rays
// -----------------------------------------------------------------------------

/**
 * @brief Simple ray (origin + direction).
 *
 * @note Direction should be normalized for consistent @ref t scaling.
 */
struct Ray
{
  Eigen::Vector3f o;  ///< Origin of the ray
  Eigen::Vector3f d;  ///< Direction of the ray (should be normalized)
};

/**
 * @brief Result of a raycast against the NavMap.
 *
 * All fields are valid only when @ref hit is true.
 */
struct RayHit
{
  bool hit{false};         ///< True if the ray hit any triangle
  size_t surface{0};       ///< Index of surface hit
  NavCelId cid{0};         ///< Id of the NavCel hit
  float t{0.0f};           ///< Distance along the ray
  Eigen::Vector3f p;       ///< World coordinates of intersection
};

// -----------------------------------------------------------------------------
// NavMap: public API
// -----------------------------------------------------------------------------

/**
 * @brief Main container for navigable surfaces, geometry, and layers.
 *
 * A @ref NavMap aggregates vertex positions, a list of @ref NavCel triangles,
 * one or more @ref Surface partitions, and a @ref LayerRegistry with arbitrary
 * per-NavCel scalar attributes (e.g., occupancy, traversability).
 *
 * Typical workflow:
 *  - Fill @ref positions, @ref navcels, and @ref surfaces.
 *  - Call @ref rebuild_geometry_accels() to compute normals, areas, BVHs, grids.
 *  - Create layers via @ref layers.add_or_get() sized to navcels.size().
 *  - Query with @ref locate_navcel(), @ref raycast(), or @ref closest_triangle().
 */
class NavMap {
public:
  Positions positions;                 ///< Vertex positions (SoA)
  std::optional<Colors> colors;        ///< Optional per-vertex colors
  std::vector<NavCel> navcels;         ///< All triangles (global indexing)
  std::vector<Surface> surfaces;       ///< Surfaces (partitions of navcels)
  LayerRegistry layers;                ///< Per-NavCel layers

  /**
   * @brief Recompute derived geometry and acceleration structures.
   *
   * Computes triangle normals and areas, builds adjacency, and builds
   * per-surface BVH and uniform seed grid. Layers are not resized.
   * Call @ref LayerRegistry::resize_all() separately if needed.
   */
  void rebuild_geometry_accels();

  /**
   * @brief Build topological adjacency between neighboring NavCels.
   *
   * Two triangles are neighbors if they share an undirected edge.
   * Called by @ref rebuild_geometry_accels().
   */
  void build_adjacency();

  /**
   * @brief Mark a vertex as updated (reserved for future cache invalidation).
   * @param pid Vertex id.
   */
  void mark_vertex_updated(PointId /*pid*/) {}

  /**
   * @brief Return the three neighbor NavCel ids of triangle @p cid.
   * @param cid NavCel id.
   * @return Array with neighbor ids or max uint32_t if boundary.
   */
  std::array<NavCelId, 3> get_neighbors(NavCelId cid) const
  {
    return {navcels[cid].neighbor[0],
      navcels[cid].neighbor[1],
      navcels[cid].neighbor[2]};
  }

  /**
   * @brief Raycast against all surfaces to find the closest hit.
   * @param o Ray origin (world).
   * @param d Ray direction (normalized).
   * @param[out] hit_cid NavCel id hit (valid if return is true).
   * @param[out] t Distance along ray (valid if return is true).
   * @param[out] hit_pt World-space intersection point.
   * @return true if any triangle was hit.
   */
  bool raycast(
    const Eigen::Vector3f & o,
    const Eigen::Vector3f & d,
    NavCelId & hit_cid,
    float & t,
    Eigen::Vector3f & hit_pt) const;

  /**
   * @brief Batched raycast.
   * @param rays Input rays.
   * @param[out] out Output hits (parallel to @p rays).
   * @param first_hit_only If true, stop at the first surface that hits.
   */
  void raycast_many(
    const std::vector<Ray> & rays,
    std::vector<RayHit> & out,
    bool first_hit_only = true) const;

  // --- Per-NavCel layer value access ---

  /**
   * @brief Read the value of a per-NavCel layer at triangle @p cid.
   * @tparam T Layer storage type.
   * @param cid NavCel id.
   * @param layer Typed layer view.
   * @return The value for triangle @p cid.
   */
  template<typename T>
  T navcel_value(NavCelId cid, const LayerView<T> & layer) const
  {
    return layer[cid];
  }

  /**
   * @brief Options for locate_navcel().
   *
   * - @ref hint_cid : starting triangle for walking (if provided).
   * - @ref hint_surface : optional surface restriction.
   * - @ref planar_eps : in-triangle barycentric tolerance.
   * - @ref height_eps : vertical tolerance when gating by AABB.
   * - @ref use_downward_ray : if grid fails, raycast down; else up.
   */
  struct LocateOpts
  {
    std::optional<NavCelId> hint_cid;    ///< Optional triangle hint
    std::optional<size_t> hint_surface;  ///< Optional surface hint
    float planar_eps = 1e-4f;            ///< Barycentric tolerance in plane
    float height_eps = 0.50f;            ///< Z tolerance for AABB gating
    bool use_downward_ray = true;        ///< Downward ray on fallback
  };

  /**
   * @brief Locate the triangle under / near a world point (convenience).
   *
   * Uses default @ref LocateOpts. See the full overload for details.
   */
  bool locate_navcel(
    const Eigen::Vector3f & p_world,
    size_t & surface_idx,
    NavCelId & cid,
    Eigen::Vector3f & bary,
    Eigen::Vector3f * hit_pt);

  /**
   * @brief Locate the triangle under / near a world point.
   *
   * Strategy:
   *  1) If @ref LocateOpts::hint_cid is provided, try walking neighbors.
   *  2) Else, try a per-surface 2D seed grid near (x,y) with planar test.
   *  3) If still not found, vertical raycast (downward or upward).
   *
   * @param p_world Query point in world coordinates.
   * @param[out] surface_idx Surface index owning the located triangle.
   * @param[out] cid Located NavCel id.
   * @param[out] bary Barycentric coords of projected point on triangle.
   * @param[out] hit_pt Optional: the projected point on the surface.
   * @param opts Tuning options (see @ref LocateOpts).
   * @return true if a triangle has been located.
   */
  bool locate_navcel(
    const Eigen::Vector3f & p_world,
    size_t & surface_idx,
    NavCelId & cid,
    Eigen::Vector3f & bary,
    Eigen::Vector3f * hit_pt,
    const LocateOpts & opts);

  /**
   * @brief Find the closest triangle to a point.
   *
   * Traverses per-surface BVHs with distance lower-bounds; returns the
   * closest triangle, the closest point on it, and the squared distance.
   *
   * @param p_world Query point in world coordinates.
   * @param[out] surface_idx Surface index of the closest triangle.
   * @param[out] cid NavCel id of the closest triangle.
   * @param[out] closest_point Closest point on that triangle.
   * @param[out] sqdist Squared distance to @p p_world.
   * @param restrict_surface If >= 0, restrict search to this surface.
   * @return true if any triangle was considered.
   */
  bool closest_triangle(
    const Eigen::Vector3f & p_world,
    size_t & surface_idx,
    NavCelId & cid,
    Eigen::Vector3f & closest_point,
    float & sqdist,
    int restrict_surface = -1) const;

private:
  // Builders and traversal helpers.

  /// @brief Build a per-surface BVH for fast ray queries.
  void build_surface_bvh(Surface & s);

  /// @brief Build a per-surface 2D uniform grid for seeding locate().
  void build_surface_grid(Surface & s, float target_cells_per_side = 64.0f);

  /**
   * @brief Raycast against a single surface BVH.
   * @return true if any triangle was hit.
   */
  bool surface_raycast(
    const Surface & s,
    const Eigen::Vector3f & o,
    const Eigen::Vector3f & d,
    NavCelId & hit_cid,
    float & t_out,
    Eigen::Vector3f & hit_pt) const;

  /**
   * @brief Walk across neighbors starting from @p start_cid.
   * @return true if @p p projects barycentrically inside some triangle.
   */
  bool locate_by_walking(
    NavCelId start_cid,
    const Eigen::Vector3f & p,
    NavCelId & cid_out,
    Eigen::Vector3f & bary,
    Eigen::Vector3f * hit_pt,
    float planar_eps);

  /**
   * @brief Attempt a grid-seeded barycentric test around (x,y).
   * @return true if a containing triangle is found.
   */
  bool locate_via_grid(
    const Surface & s,
    const Eigen::Vector3f & p,
    NavCelId & cid_out,
    Eigen::Vector3f & bary_out,
    Eigen::Vector3f * hit_pt,
    float planar_eps) const;

  /**
   * @brief BVH traversal to find the closest triangle in one surface.
   * @return true if any candidate improved the best squared distance.
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
  Eigen::Vector3f * hit_pt)
{
  return locate_navcel(p_world, surface_idx, cid, bary, hit_pt, LocateOpts{});
}

}  // namespace navmap

#endif  // NAVMAP_CORE__NAVMAP_HPP
