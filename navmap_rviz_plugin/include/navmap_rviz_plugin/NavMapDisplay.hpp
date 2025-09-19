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


#ifndef NAVMAP_RVIZ_PLUGIN__NAVMAP_DISPLAY_HPP_
#define NAVMAP_RVIZ_PLUGIN__NAVMAP_DISPLAY_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <QObject>

#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/display_context.hpp>

#include <navmap_ros_interfaces/msg/nav_map.hpp>
#include <navmap_ros_interfaces/msg/nav_map_layer.hpp>

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NAVMAP_RVIZ_PLUGIN_EXPORT __attribute__ ((dllexport))
    #define NAVMAP_RVIZ_PLUGIN_IMPORT __attribute__ ((dllimport))
  #else
    #define NAVMAP_RVIZ_PLUGIN_EXPORT __declspec(dllexport)
    #define NAVMAP_RVIZ_PLUGIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef navmap_rviz_plugin_EXPORTS
    #define NAVMAP_RVIZ_PLUGIN_PUBLIC NAVMAP_RVIZ_PLUGIN_EXPORT
  #else
    #define NAVMAP_RVIZ_PLUGIN_PUBLIC NAVMAP_RVIZ_PLUGIN_IMPORT
  #endif
  #define NAVMAP_RVIZ_PLUGIN_PUBLIC_TYPE NAVMAP_RVIZ_PLUGIN_PUBLIC
  #define NAVMAP_RVIZ_PLUGIN_LOCAL
#else
  #define NAVMAP_RVIZ_PLUGIN_PUBLIC __attribute__ ((visibility ("default")))
  #define NAVMAP_RVIZ_PLUGIN_PUBLIC_TYPE
  #define NAVMAP_RVIZ_PLUGIN_LOCAL  __attribute__ ((visibility ("hidden")))
#endif

/// Forward declarations to avoid hard coupling with OGRE headers here.
namespace Ogre
{
class SceneNode;
class ManualObject;
}  // namespace Ogre

namespace navmap_rviz_plugin
{

/**
 * @class NavMapDisplay
 * @brief RViz display for visualizing NavMap meshes and per-cell layers.
 *
 * This display renders triangle meshes from a `navmap_ros_interfaces::msg::NavMap`
 * and colorizes each triangle by a selected layer. It also supports incremental
 * layer updates via a secondary subscription to `NavMapLayer`, plus optional
 * per-triangle normal visualization.
 * This display supports color schemes per layer type: U8 layers use the fixed
 * Occupancy mapping, while float layers (F32/F64) can use either Heat or Rainbow.
 */
class NAVMAP_RVIZ_PLUGIN_PUBLIC NavMapDisplay
  : public rviz_common::MessageFilterDisplay<navmap_ros_interfaces::msg::NavMap>
{
  Q_OBJECT

  using MFDClass = rviz_common::MessageFilterDisplay<navmap_ros_interfaces::msg::NavMap>;
  using NavMapMsg = navmap_ros_interfaces::msg::NavMap;
  using NavMapLayerMsg = navmap_ros_interfaces::msg::NavMapLayer;

public:
  /// @brief Construct display and declare user-facing properties.
  NavMapDisplay();

  /// @brief Clean up OGRE objects and internal resources.
  ~NavMapDisplay() override;

  // --- RViz lifecycle ---

  /**
   * @brief Initialize the display after construction.
   *
   * Initializes QoS properties and creates the root scene node if needed.
   */
  void onInitialize() override;

  /**
   * @brief Reset display state to an empty/initial condition.
   *
   * Clears last received message, layer indices and info. Keeps or rebuilds OGRE objects as needed.
   */
  void reset() override;

  /**
   * @brief Called when the display is enabled.
   *
   * Subscribes to the layer update topic in addition to the main NavMap subscription.
   */
  void onEnable() override;

  /**
   * @brief Called when the display is disabled.
   *
   * Unsubscribes from the layer update topic and lets the base class disable main input.
   */
  void onDisable() override;

protected:
  /**
   * @brief Process an incoming full NavMap message.
   *
   * @param msg The received NavMap message.
   *
   * Updates internal state, repopulates the layer enum, rebuilds indices and triggers a redraw.
   */
  void processMessage(const NavMapMsg::ConstSharedPtr msg) override;

private Q_SLOTS:
  /**
   * @brief React to a change in the layer update topic property.
   *
   * Re-subscribes to the `NavMapLayer` topic using the configured QoS profile.
   */
  void updateLayerUpdateTopic();

  /**
   * @brief React to a change in the selected colorizing layer.
   *
   * Rebuilds geometry colors and (optionally) normals for the selected layer.
   */
  void onLayerSelectionChanged();

  /**
   * @brief Toggle drawing of per-triangle normals.
   */
  void onDrawNormalsChanged();

  /**
   * @brief Update global alpha for triangle colors.
   */
  void onAlphaChanged();

  /**
   * @brief Update the visual scale for normal lines.
   */
  void onNormalScaleChanged();

  /**
   * @brief React to a change in the selected color scheme.
   *
   * Triggers a geometry recolor (and normals refresh if enabled).
   */
  void onColorSchemeChanged();

private:
  // --- Layer update subscription (secondary input) ---

  /**
   * @brief Subscribe to the configured `NavMapLayer` topic.
   *
   * Creates a subscription using the current QoS profile. Does nothing if disabled or topic is empty.
   */
  void subscribeToLayerTopic();

  /// @brief Unsubscribe from the `NavMapLayer` topic.
  void unsubscribeToLayerTopic();

  /**
   * @brief Handler for incoming incremental layer messages.
   *
   * @param msg The received layer data for all triangles (exactly one array non-empty).
   *
   * Validates size/type, updates or inserts the layer into the last NavMap, and triggers recoloring if needed.
   */
  void incomingLayer(const NavMapLayerMsg::ConstSharedPtr & msg);

  // --- Utilities ---

  /// @brief Rebuild fast lookups from layer name to layer pointer within @ref last_msg_.
  void rebuildLayerIndex_();

  /// @brief Get the currently selected layer name from the property (empty if none).
  std::string currentSelectedLayer_() const;

  /// @brief Repopulate the layer selection enum from @ref last_msg_ while preserving the previous selection if possible.
  void repopulateLayerEnum_();

  /**
   * @brief Insert or update a layer inside the mutable copy of the last NavMap.
   *
   * @param layer The layer to insert or replace by name.
   *
   * Adds the option to the enum if it did not exist and rebuilds the layer index.
   */
  void applyOrCacheLayer_(const NavMapLayerMsg & layer);

  // --- Rendering pipeline ---

  /**
   * @brief Build or update triangle geometry and per-triangle colors.
   *
   * Recreates the manual object for triangles using the current NavMap and color scheme:
   * selected layer (U8: occupancy mapping; F32/F64: heat map) or vertex RGBA, with global alpha.
   */
  void updateGeometry_();

  /**
   * @brief Build or update the per-triangle normals manual object.
   *
   * Uses the centroid and face normal of each triangle and the configured normal scale.
   */
  void updateNormals_();

  /**
   * @brief Refresh available color-scheme options based on the current layer type.
   *
   * - U8 layers: Occupancy (fixed).
   * - F32/F64 layers: Heat or Rainbow.
   * Preserves the previous selection when still applicable.
   */
  void updateColorSchemeOptions_();

private:
  // ---- User-facing properties ----

  /// @brief Selected layer to colorize triangles.
  rviz_common::properties::EnumProperty * layer_property_{nullptr};

  /// @brief Topic for incremental layer updates (`NavMapLayer`).
  rviz_common::properties::RosTopicProperty * layer_topic_property_{nullptr};

  /// @brief QoS profile for the layer update subscription.
  rviz_common::properties::QosProfileProperty * layer_profile_property_{nullptr};

  /// @brief Color mapping for the active layer (U8: Occupancy; F32/F64: Heat or Rainbow).
  rviz_common::properties::EnumProperty * color_scheme_property_{nullptr};

  /// @brief Toggle drawing of per-triangle normals.
  rviz_common::properties::BoolProperty * draw_normals_property_{nullptr};

  /// @brief Scale for normal segments.
  rviz_common::properties::FloatProperty * normal_scale_property_{nullptr};

  /// @brief Global transparency for triangle colors.
  rviz_common::properties::FloatProperty * alpha_property_{nullptr};

  /// @brief Read-only info about the current NavMap (counts, etc.).
  rviz_common::properties::StringProperty * info_property_{nullptr};

  // ---- Layer QoS and subscription ----

  /// @brief QoS used for the layer update subscription.
  rclcpp::QoS layer_profile_{rclcpp::QoS(5)};

  /// @brief Subscription to incremental `NavMapLayer` messages.
  rclcpp::Subscription<NavMapLayerMsg>::SharedPtr layer_subscription_;

  /// @brief Timestamp when the layer subscription was created (diagnostics).
  rclcpp::Time layer_subscription_start_time_;

  // ---- Data state ----

  /// @brief Mutable copy of the last full NavMap message to integrate layer updates.
  NavMapMsg::SharedPtr last_msg_;

  /// @brief Fast lookup from layer name to layer pointer within @ref last_msg_.
  std::unordered_map<std::string, const NavMapLayerMsg *> layers_by_name_;

  // ---- OGRE scene objects ----

  /// @brief Root node used to attach display geometry.
  Ogre::SceneNode * root_node_{nullptr};

  /// @brief Manual object for triangle geometry and colors.
  Ogre::ManualObject * triangles_obj_{nullptr};

  /// @brief Manual object for per-triangle normal segments.
  Ogre::ManualObject * normals_obj_{nullptr};

  std::uint64_t navmap_msg_count_{0};
  std::uint64_t layer_update_count_{0};
  rclcpp::Time last_navmap_stamp_;
  rclcpp::Time last_layer_stamp_;
};

}  // namespace navmap_rviz_plugin

#endif  // NAVMAP_RVIZ_PLUGIN__NAVMAP_DISPLAY_HPP_
