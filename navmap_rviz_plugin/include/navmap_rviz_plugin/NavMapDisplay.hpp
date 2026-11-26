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


#ifndef NAVMAP_RVIZ_PLUGIN__NAVMAPDISPLAY_HPP_
#define NAVMAP_RVIZ_PLUGIN__NAVMAPDISPLAY_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "QObject"
#include "OgreHardwareVertexBuffer.h"
#include "OgreMesh.h"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/display_context.hpp"
#include "navmap_ros_interfaces/msg/nav_map.hpp"
#include "navmap_ros_interfaces/msg/nav_map_layer.hpp"
#include "navmap_ros_interfaces/msg/nav_map_patch.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"

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

// Forward declarations to avoid hard coupling here
namespace Ogre
{
class SceneNode;
class ManualObject;
class Entity;
class Mesh;
class HardwareVertexBuffer;
}

namespace navmap_rviz_plugin
{

inline navmap::NavMap received_navmap;
/// The last map a NavMapDisplay showed, and whether received_navmap is older than it.
inline navmap_ros_interfaces::msg::NavMap::SharedPtr received_msg;
inline bool received_stale{false};

/// The NavMap last shown (for the tools: raycast on click), built from it when first needed: a
/// map or patch that arrives costs nothing here.
inline navmap::NavMap & current_received_navmap()
{
  if (received_stale && received_msg) {
    received_navmap = navmap_ros::from_msg(*received_msg);
    received_stale = false;
  }
  return received_navmap;
}

class NAVMAP_RVIZ_PLUGIN_PUBLIC NavMapDisplay
  : public rviz_common::MessageFilterDisplay<navmap_ros_interfaces::msg::NavMap>
{
  Q_OBJECT

  using MFDClass = rviz_common::MessageFilterDisplay<navmap_ros_interfaces::msg::NavMap>;
  using NavMapMsg = navmap_ros_interfaces::msg::NavMap;
  using NavMapLayerMsg = navmap_ros_interfaces::msg::NavMapLayer;
  using NavMapPatchMsg = navmap_ros_interfaces::msg::NavMapPatch;

public:
  NavMapDisplay();
  ~NavMapDisplay() override;

  void onInitialize() override;
  void reset() override;
  void onEnable() override;
  void onDisable() override;

protected:
  void processMessage(const NavMapMsg::ConstSharedPtr msg) override;

private Q_SLOTS:
  void updateLayerUpdateTopic();
  void updatePatchTopic();
  void onLayerSelectionChanged();
  void onDrawNormalsChanged();
  void onDrawEdgesChanged();
  void onDrawLinksChanged();
  void onAlphaChanged();
  void onNormalScaleChanged();
  void onColorSchemeChanged();

private:
  // Subscriptions
  void subscribeToLayerTopic();
  void unsubscribeToLayerTopic();
  void incomingLayer(const NavMapLayerMsg::ConstSharedPtr & msg);
  void subscribeToPatchTopic();
  void unsubscribeToPatchTopic();
  void incomingPatch(const NavMapPatchMsg::ConstSharedPtr & msg);

  // Data helpers
  void rebuildLayerIndex_();
  std::string currentSelectedLayer_() const;
  void repopulateLayerEnum_();
  void applyOrCacheLayer_(const NavMapLayerMsg & layer);

  // Rendering
  void updateGeometry_();            // Build geometry (first time) and update colors.
  void updateNormals_();             // Rebuild normals debug object.
  void updateEdges_();               // Rebuild the edges of the NavCels (one line per edge).
  void updateLinks_();               // Rebuild the links between non-adjacent NavCels.
  void updateColorSchemeOptions_();  // Refresh color-scheme choices for current layer.

  // The mesh, in chunks of kChunkCels NavCels: a patch rebuilds only the chunks it touches.
  // Each chunk: a static mesh, its entity, a dynamic colour buffer and its edges.
  struct Chunk
  {
    Ogre::Entity * entity{nullptr};
    Ogre::SharedPtr<Ogre::Mesh> mesh;
    Ogre::HardwareVertexBufferSharedPtr colour;
    Ogre::ManualObject * edges{nullptr};
  };
  static constexpr std::size_t kChunkCels = 65536;

  // How the selected layer is coloured, computed once for all the chunks
  struct ColourCtx
  {
    const NavMapLayerMsg * layer{nullptr};
    bool vertex_colours{false};
    bool is_u8{false};
    bool logodds{false};
    bool rainbow{false};
    float min_val{0.0f}, max_val{0.0f}, alpha{1.0f};
  };

  void ensureMeshBuilt_();           // Builds the chunks missing from last_msg_.
  void destroyMesh_();               // Destroys every chunk.
  void buildChunk_(std::size_t k);   // (Re)builds the geometry of chunk k.
  void destroyChunk_(Chunk & chunk);
  ColourCtx colourContext_() const;
  void colourChunk_(std::size_t k, const ColourCtx & ctx);
  void edgesChunk_(std::size_t k);
  void updateColorsOnly_();          // Recolours every chunk (no geometry rebuild).

private:
  // ---- RViz properties ----
  rviz_common::properties::EnumProperty * layer_property_{nullptr};
  rviz_common::properties::RosTopicProperty * layer_topic_property_{nullptr};
  rviz_common::properties::QosProfileProperty * layer_profile_property_{nullptr};
  rviz_common::properties::EnumProperty * color_scheme_property_{nullptr};
  rviz_common::properties::BoolProperty * draw_normals_property_{nullptr};
  rviz_common::properties::BoolProperty * draw_links_property_{nullptr};
  rviz_common::properties::BoolProperty * draw_edges_property_{nullptr};
  rviz_common::properties::ColorProperty * edge_color_property_{nullptr};
  rviz_common::properties::FloatProperty * normal_scale_property_{nullptr};
  rviz_common::properties::FloatProperty * alpha_property_{nullptr};
  rviz_common::properties::StringProperty * info_property_{nullptr};

  // ---- QoS and layer subscription ----
  rclcpp::QoS layer_profile_{rclcpp::QoS(5)};
  rclcpp::Subscription<NavMapLayerMsg>::SharedPtr layer_subscription_;
  rclcpp::Time layer_subscription_start_time_;
  rviz_common::properties::RosTopicProperty * patch_topic_property_{nullptr};
  rclcpp::Subscription<NavMapPatchMsg>::SharedPtr patch_subscription_;
  std::uint64_t patch_count_{0};

  // ---- Status counters ----
  std::uint64_t navmap_msg_count_{0};
  std::uint64_t layer_update_count_{0};
  rclcpp::Time   last_navmap_stamp_;
  rclcpp::Time   last_layer_stamp_;

  // ---- Data state ----
  NavMapMsg::SharedPtr last_msg_;
  std::unordered_map<std::string, const NavMapLayerMsg *> layers_by_name_;

  // ---- OGRE scene objects ----
  Ogre::SceneNode * root_node_{nullptr};
  Ogre::ManualObject * normals_obj_{nullptr};
  Ogre::ManualObject * links_obj_{nullptr};

  std::vector<Chunk> chunks_;
  std::string material_;   ///< vertex-colour material shared by the chunks
};

}  // namespace navmap_rviz_plugin

#endif  // NAVMAP_RVIZ_PLUGIN__NAVMAPDISPLAY_HPP_
