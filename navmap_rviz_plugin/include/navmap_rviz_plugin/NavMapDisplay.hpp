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


#ifndef NAVMAP_RVIZ_PLUGIN__NAVMAPDISPLAY_HPP_
#define NAVMAP_RVIZ_PLUGIN__NAVMAPDISPLAY_HPP_

/**
 * @file navmapdisplay.hpp
 * @brief RViz display plugin for visualizing NavMap messages.
 *
 * This display renders NavMap surfaces (triangular meshes) and optional layers
 * published as `navmap_ros_interfaces::msg::NavMap`. It supports switching
 * between layers, adjusting transparency, and visualizing per-triangle normals.
 */

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>

#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/objects/movable_text.hpp>

#include <Ogre.h>

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include <navmap_ros_interfaces/msg/nav_map.hpp>
#include <navmap_ros_interfaces/msg/nav_map_layer.hpp>
#include <navmap_ros_interfaces/msg/nav_map_surface.hpp>

namespace navmap_rviz_plugin
{

/**
 * @class NavMapDisplay
 * @brief RViz plugin to display NavMap messages as triangle meshes.
 *
 * This class inherits from `rviz_common::MessageFilterDisplay` to subscribe
 * to and process `navmap_ros_interfaces::msg::NavMap` messages. It renders
 * the geometry in the RViz scene using Ogre primitives, with optional coloring
 * by selected layers and visualization of per-triangle normals.
 *
 * @note Typical usage: add the plugin in RViz, choose the "NavMap" display,
 *       and select the desired layer to visualize.
 */
class NavMapDisplay : public rviz_common::MessageFilterDisplay<navmap_ros_interfaces::msg::NavMap>
{
public:
  /// @brief Construct an empty display.
  NavMapDisplay();

  /// @brief Destructor. Frees Ogre objects and scene nodes.
  ~NavMapDisplay() override;

protected:
  /**
   * @brief Called once after the display is created.
   *
   * Initializes Ogre scene nodes, creates properties in the RViz panel,
   * and sets up defaults for layer selection and rendering options.
   */
  void onInitialize() override;

  /**
   * @brief Reset the display to its initial state.
   *
   * Clears all scene objects and resets internal data structures.
   * Called when RViz resets the display.
   */
  void reset() override;

  /**
   * @brief Process an incoming NavMap message.
   *
   * Stores the message, rebuilds geometry buffers, and updates the
   * RViz scene with the new mesh and optional layer coloring.
   *
   * @param[in] msg Shared pointer to the received NavMap message.
   */
  void processMessage(const navmap_ros_interfaces::msg::NavMap::ConstSharedPtr msg) override;

private:
  // --------- Helpers ---------

  /**
   * @brief Remove all Ogre objects and clear the scene.
   *
   * Used before rebuilding geometry or when resetting the display.
   */
  void clearScene();

  /**
   * @brief Build Ogre ManualObjects to render triangles and normals.
   *
   * Iterates over surfaces and layers in the last received NavMap,
   * creating geometry buffers with the appropriate coloring scheme.
   */
  void rebuildGeometry();

  /**
   * @brief Update the choices available in the "Layer" property.
   *
   * Extracts available layers from the last received message and
   * populates the property dropdown accordingly.
   */
  void updateLayerChoices();

  /**
   * @brief Convert an 8-bit occupancy value to an Ogre color.
   *
   * @param[in] v Raw layer value in the range [0,255].
   * @return Corresponding `Ogre::ColourValue` (grayscale).
   */
  Ogre::ColourValue colorFromU8(uint8_t v) const;

  /**
   * @brief Convert a scalar value to a heatmap color.
   *
   * @param[in] value     Input scalar value.
   * @param[in] max_value Maximum expected value for scaling.
   * @return Color from blue (low) to red (high).
   */
  Ogre::ColourValue colorFromHeat(float value, float max_value) const;

  /**
   * @brief Callback triggered when any property changes.
   *
   * Rebuilds geometry or updates the scene as needed.
   */
  void onPropertyChanged();

  // --------- Properties (user-configurable in RViz) ---------

  rviz_common::properties::EnumProperty * layer_property_;          ///< Selected layer to colorize.
  rviz_common::properties::BoolProperty * draw_normals_property_;   ///< Toggle display of per-triangle normals.
  rviz_common::properties::FloatProperty * normal_scale_property_;  ///< Length scale for drawn normals.
  rviz_common::properties::FloatProperty * alpha_property_;         ///< Global transparency factor [0,1].
  rviz_common::properties::StringProperty * info_property_;          ///< Read-only info field (e.g., statistics).

  // --------- Scene graph ---------

  Ogre::ManualObject * triangles_obj_;  ///< Ogre object storing the rendered triangles.
  Ogre::ManualObject * normals_obj_;    ///< Ogre object storing line segments for normals.
  Ogre::SceneNode * root_node_;         ///< Root scene node for all geometry in this display.

  // --------- Data ---------

  navmap_ros_interfaces::msg::NavMap::ConstSharedPtr last_msg_;  ///< Last received NavMap message.
  std::unordered_map<std::string, const navmap_ros_interfaces::msg::NavMapLayer *>
  layers_by_name_;   ///< Mapping from layer name to layer pointer for quick lookup.
};

} // namespace navmap_rviz_plugin

#endif  // NAVMAP_RVIZ_PLUGIN__NAVMAPDISPLAY_HPP_
