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

/**
 * @file NavMap.hpp
 * @brief Core data structures and API for NavMap: triangle-based navigable
 *        surfaces, attribute layers, and fast spatial queries.
 *
 * This header declares:
 *  - Compact SoA vertex storage (Positions, optional Colors).
 *  - Dynamic per-vertex layers with type-safe views (LayerRegistry/LayerView).
 *  - Triangle cells (NavCel) with adjacency, normals and areas.
 *  - Per-surface acceleration structures (BVH + XY uniform grid).
 *  - High-level queries: raycast, locate by position, closest triangle.
 */

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

/**
 * @brief Index type for vertices in the global position/color arrays.
 */
using PointId = uint32_t;
/**
 * @brief Index type for triangle cells (NavCel).
 */
using NavCelId = uint32_t;

// -----------------------------------------------------------------------------
// Core data arrays
// -----------------------------------------------------------------------------

/**
 * @brief Structure-of-Arrays (SoA) storage for vertex positions.
 *
 * Splitting x/y/z into separate arrays improves cache locality and enables
 * tight packing for large maps. Use @ref at() to load a 3D vector by id.
 */
struct Positions
{
  /// X coordinates per vertex id.
  std::vector<float> x;
  /// Y coordinates per vertex id.
  std::vector<float> y;
  /// Z coordinates per vertex id.
  std::vector<float> z;

  /**
   * @brief Number of stored vertices.
   */
  inline size_t size() const {return x.size();}

  /**
   * @brief Fetch the 3D position of a vertex.
   * @param id Vertex id to read.
   * @return Position as Eigen::Vector3f.
   */
  inline Eigen::Vector3f at(PointId id) const
  {
    return {x[id], y[id], z[id]};
  }
};

/**
 * @brief Optional per-vertex RGBA color buffer (8-bit channels).
 *
 * If present, the color of a NavCel can be defined as the average of its
 * three vertex colors.
 */
struct Colors
{
  std::vector<uint8_t> r;  ///< Red channel per vertex id.
  std::vector<uint8_t> g;  ///< Green channel per vertex id.
  std::vector<uint8_t> b;  ///< Blue channel per vertex id.
  std::vector<uint8_t> a;  ///< Alpha channel per vertex id.
};

// -----------------------------------------------------------------------------
// Layers (dynamic per-vertex attributes)
// -----------------------------------------------------------------------------

/**
 * @brief Supported primitive types for dynamic layers.
 */
enum class LayerType : uint8_t { U8 = 0, F32 = 1, F64 = 2 };

/**
 * @brief Type-erased base view over a per-vertex dynamic layer.
 *
 * Concrete views are templated as @ref LayerView<T>.
 */
struct LayerViewBase
{
  virtual ~LayerViewBase() = default;

  /// @brief Element type identifier of the layer.
  virtual LayerType type() const = 0;
  /// @brief Layer symbolic name.
  virtual const std::string & name() const = 0;
  /// @brief Number of elements (should match Positions::size()).
  virtual size_t size() const = 0;
};

/**
 * @brief Typed view over a dynamic per-vertex attribute layer.
 *
 * Provides [] access by @ref PointId and exposes the underlying std::vector.
 *
 * @tparam T Element type (must match @ref LayerType).
 */
template<typename T>
struct LayerView : LayerViewBase
{
  std::string name_;       ///< Layer name (unique key in the registry).
  std::vector<T> data_;    ///< Contiguous storage (one value per vertex).
  LayerType type_;         ///< Declared type of this layer.

  /**
   * @brief Construct a typed layer with given size and name.
   * @param name Unique layer name.
   * @param npoints Number of vertices (capacity of the layer).
   * @param t Declared layer type (should be consistent with T).
   */
  LayerView(std::string name, size_t npoints, LayerType t)
  : name_(std::move(name)), data_(npoints), type_(t) {}

  /// @copydoc LayerViewBase::type()
  LayerType type() const override {return type_;}
  /// @copydoc LayerViewBase::name()
  const std::string & name() const override {return name_;}
  /// @copydoc LayerViewBase::size()
  size_t size() const override {return data_.size();}

  /// @brief Mutable element access by vertex id.
  T & operator[](PointId pid) {return data_[pid];}
  /// @brief Const element access by vertex id.
  const T & operator[](PointId pid) const {return data_[pid];}
  /// @brief Mutable reference to underlying storage.
  std::vector<T> & data() {return data_;}
  /// @brief Const reference to underlying storage.
  const std::vector<T> & data() const {return data_;}
};

/**
 * @brief Registry of named dynamic layers, with type-erased storage.
 *
 * The registry owns shared pointers to layer views. Clients can create
 * or retrieve typed views via @ref add_or_get<T>() and list available
 * layers by name via @ref list().
 */
class LayerRegistry {
public:
  /**
   * @brief Get (or create) a typed layer view.
   *
   * If a layer with @p name already exists, returns it (cast to T).
   * Otherwise, creates a new layer with @p npoints elements.
   *
   * @tparam T Element type to access (must match @p type).
   * @param name Unique layer name.
   * @param npoints Number of vertices (layer length).
   * @param type Element type identifier (kept for introspection).
   * @return Shared pointer to a typed @ref LayerView<T>.
   */
  template<typename T>
  std::shared_ptr<LayerView<T>> add_or_get(
    const std::string & name,
    size_t npoints,
    LayerType type)
  {
    auto it = layers_.find(name);
    if (it != layers_.end()) {
      return std::dynamic_pointer_cast<LayerView<T>>(it->second);
    }
    auto view = std::make_shared<LayerView<T>>(name, npoints, type);
    layers_[name] = view;
    return view;
  }

  /**
   * @brief Get a type-erased layer view by name.
   * @param name Layer name.
   * @return Shared pointer to base view or nullptr if not found.
   */
  std::shared_ptr<LayerViewBase> get(const std::string & name) const
  {
    auto it = layers_.find(name);
    return it == layers_.end() ? nullptr : it->second;
  }

  /**
   * @brief List all registered layer names.
   * @return Vector of layer names.
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

private:
  /// @brief Storage of layers by name (type-erased views).
  std::unordered_map<std::string, std::shared_ptr<LayerViewBase>> layers_;
};

// -----------------------------------------------------------------------------
// Mesh cells and acceleration
// -----------------------------------------------------------------------------

/**
 * @brief Triangle cell (NavCel) defined by 3 vertex ids.
 *
 * Stores topology (neighbor ids per edge), geometric caches (unit normal
 * and area), and a bitmask for dirty layer aggregation.
 */
struct NavCel
{
  /// Indices of the three vertices (counter-clockwise convention).
  PointId v[3]{0, 0, 0};
  /// Unit-length triangle normal in world coordinates.
  Eigen::Vector3f normal{0.0f, 0.0f, 1.0f};
  /// Triangle area (computed from positions).
  float area{0.0f};
  /**
   * @brief Neighbor triangle ids across each edge.
   *
   * neighbor[i] is the triangle across edge (v[i], v[(i+1)%3]).
   * A value of max<uint32_t>() means border (no neighbor).
   */
  NavCelId neighbor[3]{
    std::numeric_limits<uint32_t>::max(),
    std::numeric_limits<uint32_t>::max(),
    std::numeric_limits<uint32_t>::max()
  };
  /// Bitmask: per-layer dirty flags (user-managed).
  uint32_t layer_dirty_mask{0};
};

/**
 * @brief Node of a binary BVH over triangle primitives (per surface).
 */
struct BVHNode
{
  AABB box;     ///< Node bounding box.
  int left{-1}; ///< Left child index (or -1).
  int right{-1};///< Right child index (or -1).
  int start{0}; ///< Leaf: start index in @ref Surface::prim_indices.
  int count{0}; ///< Leaf: number of primitives in this node.
  /// @brief Whether this node is a leaf (count > 0).
  bool is_leaf() const {return count > 0;}
};

/**
 * @brief Lightweight 2D uniform grid for seeding locate() without a hint.
 *
 * Buckets triangle ids by XY cells to quickly find candidates near a query
 * position. Used as a fast pre-filter prior to precise checks.
 */
struct UniformGrid2D
{
  Eigen::Vector2f origin{0.0f, 0.0f};     ///< World-space origin of cell (0,0).
  Eigen::Vector2f cell_size{0.25f, 0.25f};///< Cell size in meters (dx, dy).
  int nx{0};                               ///< Number of cells along X.
  int ny{0};                               ///< Number of cells along Y.
  std::vector<std::vector<int>> buckets;   ///< Per-cell lists of triangle ids.

  /**
   * @brief Whether the grid is built and usable.
   */
  inline bool valid() const
  {
    return nx > 0 && ny > 0 && !buckets.empty();
  }

  /**
   * @brief Convert (ix, iy) to linear index or -1 if out of range.
   */
  inline int index(int ix, int iy) const
  {
    if (ix < 0 || iy < 0 || ix >= nx || iy >= ny) {
      return -1;
    }
    return iy * nx + ix;
  }

  /**
   * @brief Compute the grid cell coordinates that contain point @p p (XY).
   * @param p XY position (Z ignored).
   * @return Integer cell coordinates (ix, iy).
   */
  inline Eigen::Vector2i cell_of(const Eigen::Vector2f & p) const
  {
    int ix = static_cast<int>(std::floor((p.x() - origin.x()) / cell_size.x()));
    int iy = static_cast<int>(std::floor((p.y() - origin.y()) / cell_size.y()));
    return {ix, iy};
  }
};

/**
 * @brief A navigable surface (e.g., a floor) with its own acceleration data.
 *
 * A surface contains a subset of triangle ids, an AABB over those triangles,
 * a BVH for ray and nearest queries, and a 2D XY grid for quick seeding.
 * The @ref frame_id can be used to relate the surface to a TF frame.
 */
struct Surface
{
  std::string frame_id;            ///< Optional frame id (for ROS integration).
  std::vector<NavCelId> navcels;   ///< Triangle ids that belong to this surface.
  AABB aabb;                       ///< Bounding box of all triangles in surface.
  std::vector<int> prim_indices;   ///< Primitive ids (NavCel ids) in BVH order.
  std::vector<BVHNode> bvh;        ///< Compact binary BVH nodes.
  UniformGrid2D grid;              ///< XY uniform grid buckets.
};

// -----------------------------------------------------------------------------
// NavMap: public API
// -----------------------------------------------------------------------------

/**
 * @brief Main map container with geometry, layers and query API.
 *
 * The NavMap owns the global vertex buffers, per-triangle data and
 * a set of surfaces (each with its own accelerations). Typical usage:
 *  1) Fill positions, colors (optional), navcels and surfaces.
 *  2) Call @ref rebuild_geometry_accels().
 *  3) Perform queries: @ref locate_navcel(), @ref raycast(), @ref closest_triangle().
 */
class NavMap {
public:
  Positions positions;              ///< Global vertex positions (SoA).
  std::optional<Colors> colors;     ///< Optional RGBA colors per vertex.
  std::vector<NavCel> navcels;      ///< All triangle cells.
  std::vector<Surface> surfaces;    ///< Set of surfaces (e.g., floors).
  LayerRegistry layers;             ///< Dynamic per-vertex attribute layers.

  /**
   * @brief Recompute per-triangle normals/areas, adjacency and per-surface
   *        accelerations (BVH + uniform grid).
   *
   * Must be called after changing geometry (positions, navcels or surfaces).
   */
  void rebuild_geometry_accels();

  /**
   * @brief Build triangle adjacency across shared edges.
   *
   * Usually invoked by @ref rebuild_geometry_accels(). Safe to call directly
   * if you update topology manually.
   */
  void build_adjacency();

  /**
   * @brief Notify that a vertex attribute changed (hook for future updates).
   * @param pid Vertex id that changed.
   * @note Currently a no-op; kept for API stability.
   */
  void mark_vertex_updated(PointId /*pid*/) {}

  /**
   * @brief Return the three neighbor triangle ids of a NavCel.
   * @param cid Triangle id.
   * @return Array [n0, n1, n2] (max<uint32_t>() if border).
   */
  std::array<NavCelId, 3> get_neighbors(NavCelId cid) const
  {
    return {navcels[cid].neighbor[0],
      navcels[cid].neighbor[1],
      navcels[cid].neighbor[2]};
  }

  /**
   * @brief Raycast against all surfaces.
   *
   * @param o Ray origin (world).
   * @param d Ray direction (world).
   * @param hit_cid Output: id of the hit triangle.
   * @param t Output: parametric distance along the ray.
   * @param hit_pt Output: world hit position.
   * @return True if the ray hits any triangle.
   */
  bool raycast(
    const Eigen::Vector3f & o,
    const Eigen::Vector3f & d,
    NavCelId & hit_cid,
    float & t,
    Eigen::Vector3f & hit_pt) const;

  /**
   * @brief Compute the mean of a per-vertex layer over a triangle's vertices.
   *
   * @tparam T Layer element type.
   * @param cid Triangle id.
   * @param layer Typed layer view with at least 3 entries for the triangle's vertices.
   * @return Arithmetic mean of the three vertex values.
   */
  template<typename T>
  T navcel_mean(NavCelId cid, const LayerView<T> & layer) const
  {
    const auto & c = navcels[cid];
    return (layer[c.v[0]] + layer[c.v[1]] + layer[c.v[2]]) / static_cast<T>(3);
  }

  /**
   * @brief Options for locating a triangle under/near a world position.
   *
   * - hint_cid: Optional starting triangle for a local walk.
   * - hint_surface: Optional surface index hint.
   * - planar_eps: Tolerance for barycentric inside-test after planar projection.
   * - height_eps: (Optional) height preference band around the query.
   * - use_downward_ray: If true, try downward ray first, then upward.
   */
  struct LocateOpts
  {
    std::optional<NavCelId> hint_cid;
    std::optional<size_t> hint_surface;
    float planar_eps = 1e-4f;
    float height_eps = 0.50f;
    bool use_downward_ray = true;
  };

  /**
   * @brief Locate a NavCel under/near a world position (convenience overload).
   *
   * Uses default @ref LocateOpts. On success, also returns barycentric
   * coordinates and the hit/projection point if @p hit_pt is non-null.
   */
  bool locate_navcel(
    const Eigen::Vector3f & p_world,
    size_t & surface_idx,
    NavCelId & cid,
    Eigen::Vector3f & bary,
    Eigen::Vector3f * hit_pt);

  /**
   * @brief Locate a NavCel under/near a world position with custom options.
   *
   * The algorithm may try (in order): walk from hint, grid-based seed over
   * XY cells, and/or vertical raycasts (down/up) to disambiguate stacked
   * surfaces (e.g., multi-floor buildings).
   *
   * @param p_world Query point in world coordinates.
   * @param surface_idx Output surface index that owns the found triangle.
   * @param cid Output triangle id.
   * @param bary Output barycentric coordinates (u, v, w).
   * @param hit_pt Optional output: contact/projection point on the triangle.
   * @param opts Algorithm options (see @ref LocateOpts).
   * @return True if a suitable triangle is found.
   */
  bool locate_navcel(
    const Eigen::Vector3f & p_world,
    size_t & surface_idx,
    NavCelId & cid,
    Eigen::Vector3f & bary,
    Eigen::Vector3f * hit_pt,
    const LocateOpts & opts);

  /**
   * @brief Find the triangle closest to a world position.
   *
   * Optionally restricts the search to a single surface. Returns the closest
   * point on that triangle and the squared distance.
   *
   * @param p_world Query position.
   * @param surface_idx Output surface index.
   * @param cid Output triangle id.
   * @param closest_point Output closest point on the triangle.
   * @param sqdist Output squared distance to the triangle.
   * @param restrict_surface If >= 0, restrict search to this surface index.
   * @return True if any triangle is found.
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

  /**
   * @brief Build the BVH for a surface (nodes + primitive order).
   * @param s Surface to update.
   */
  void build_surface_bvh(Surface & s);

  /**
   * @brief Build a coarse XY uniform grid for fast seeding.
   * @param s Surface to update.
   * @param target_cells_per_side Desired grid resolution (approximate).
   */
  void build_surface_grid(Surface & s, float target_cells_per_side = 64.0f);

  /**
   * @brief BVH-accelerated raycast against a single surface.
   *
   * @param s Surface to test.
   * @param o Ray origin.
   * @param d Ray direction.
   * @param hit_cid Output triangle id on hit.
   * @param t_out Output ray parameter.
   * @param hit_pt Output world hit position.
   * @return True if the ray hits any triangle of the surface.
   */
  bool surface_raycast(
    const Surface & s,
    const Eigen::Vector3f & o,
    const Eigen::Vector3f & d,
    NavCelId & hit_cid,
    float & t_out,
    Eigen::Vector3f & hit_pt) const;

  /**
   * @brief Walk across adjacent triangles starting from a hint, projecting
   *        @p p to each triangle plane and testing barycentric inclusion.
   *
   * @param start_cid Starting triangle id.
   * @param p Query position.
   * @param cid_out Output triangle id where point lies (if found).
   * @param bary Output barycentric coordinates.
   * @param hit_pt Optional output: projected point on the triangle.
   * @param planar_eps Barycentric tolerance.
   * @return True if a containing triangle is found by walking.
   */
  bool locate_by_walking(
    NavCelId start_cid,
    const Eigen::Vector3f & p,
    NavCelId & cid_out,
    Eigen::Vector3f & bary,
    Eigen::Vector3f * hit_pt,
    float planar_eps);

  /**
   * @brief Seed locate by querying the XY uniform grid and testing candidates.
   *
   * @param s Surface to query.
   * @param p Query position.
   * @param cid_out Output triangle id.
   * @param bary_out Output barycentric coordinates.
   * @param hit_pt Optional output: projected point on the triangle.
   * @param planar_eps Barycentric tolerance.
   * @return True if a containing triangle is found in nearby buckets.
   */
  bool locate_via_grid(
    const Surface & s,
    const Eigen::Vector3f & p,
    NavCelId & cid_out,
    Eigen::Vector3f & bary_out,
    Eigen::Vector3f * hit_pt,
    float planar_eps) const;

  /**
   * @brief Search the closest triangle in a surface using BVH pruning.
   *
   * @param s Surface to search.
   * @param p Query point.
   * @param cid Output triangle id.
   * @param q Output closest point on the triangle.
   * @param best_sq Input/Output: current best squared distance (updated if improved).
   * @return True if any candidate improved @p best_sq.
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
