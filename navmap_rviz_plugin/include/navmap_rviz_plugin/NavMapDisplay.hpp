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


#ifndef NAVMAP_RVIZ_PLUGIN__NAVMAP_DISPLAY_HPP_
#define NAVMAP_RVIZ_PLUGIN__NAVMAP_DISPLAY_HPP_

#include <cstdint>
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

class NAVMAP_RVIZ_PLUGIN_PUBLIC NavMapDisplay
  : public rviz_common::MessageFilterDisplay<navmap_ros_interfaces::msg::NavMap>
{
  Q_OBJECT

  using MFDClass = rviz_common::MessageFilterDisplay<navmap_ros_interfaces::msg::NavMap>;
  using NavMapMsg = navmap_ros_interfaces::msg::NavMap;
  using NavMapLayerMsg = navmap_ros_interfaces::msg::NavMapLayer;

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
  void onLayerSelectionChanged();
  void onDrawNormalsChanged();
  void onAlphaChanged();
  void onNormalScaleChanged();
  void onColorSchemeChanged();

private:
  // Subscriptions
  void subscribeToLayerTopic();
  void unsubscribeToLayerTopic();
  void incomingLayer(const NavMapLayerMsg::ConstSharedPtr & msg);

  // Data helpers
  void rebuildLayerIndex_();
  std::string currentSelectedLayer_() const;
  void repopulateLayerEnum_();
  void applyOrCacheLayer_(const NavMapLayerMsg & layer);

  // Rendering
  void updateGeometry_();            // Build geometry (first time) and update colors.
  void updateNormals_();             // Rebuild normals debug object.
  void updateColorSchemeOptions_();  // Refresh color-scheme choices for current layer.

  // Mesh/Entity path (Option B)
  void ensureMeshBuilt_();           // Build static mesh once from last_msg_.
  void destroyMesh_();               // Destroy entity/mesh and related buffers.
  void updateColorsOnly_();          // Overwrite color buffer (fast path, no geometry rebuild).

private:
  // ---- RViz properties ----
  rviz_common::properties::EnumProperty * layer_property_{nullptr};
  rviz_common::properties::RosTopicProperty * layer_topic_property_{nullptr};
  rviz_common::properties::QosProfileProperty * layer_profile_property_{nullptr};
  rviz_common::properties::EnumProperty * color_scheme_property_{nullptr};
  rviz_common::properties::BoolProperty * draw_normals_property_{nullptr};
  rviz_common::properties::FloatProperty * normal_scale_property_{nullptr};
  rviz_common::properties::FloatProperty * alpha_property_{nullptr};
  rviz_common::properties::StringProperty * info_property_{nullptr};

  // ---- QoS and layer subscription ----
  rclcpp::QoS layer_profile_{rclcpp::QoS(5)};
  rclcpp::Subscription<NavMapLayerMsg>::SharedPtr layer_subscription_;
  rclcpp::Time layer_subscription_start_time_;

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

  // Option B: static mesh + entity + dynamic colour buffer
  Ogre::Entity * entity_{nullptr};
  Ogre::SharedPtr<Ogre::Mesh> mesh_;  // Ogre::MeshPtr
  Ogre::HardwareVertexBufferSharedPtr colour_vbuf_;
  unsigned short colour_vbuf_source_{0};   // which stream holds VES_DIFFUSE
  bool mesh_built_{false};
};

}  // namespace navmap_rviz_plugin

#endif  // NAVMAP_RVIZ_PLUGIN__NAVMAP_DISPLAY_HPP_
