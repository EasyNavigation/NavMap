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


#include <OgreRoot.h>
#include <OgreRenderSystem.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h>
#include <OgreMesh.h>
#include <OgreMeshManager.h>
#include <OgreMaterialManager.h>
#include <OgreSubMesh.h>
#include <OgreTechnique.h>
#include <OgrePass.h>

#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"

#include "navmap_rviz_plugin/NavMapDisplay.hpp"
namespace
{

// Shared between repopulateLayerEnum_() (offers it in the "Layer" dropdown)
// and updateColorsOnly_() (recognizes it once selected) -- kept as a single
// constant so the two can't drift apart.
const char * const kVertexColorLayerName = "Color (vertex RGBA)";

inline void hsv2rgb(float H, float S, float V, float & R, float & G, float & B)
{
  const float C = V * S;
  const float X = C * (1.0f - std::fabs(std::fmod(H / 60.0f, 2.0f) - 1.0f));
  const float m = V - C;

  float r1 = 0.f, g1 = 0.f, b1 = 0.f;
  if      (H < 60.f) {r1 = C; g1 = X; b1 = 0.f;} else if (H < 120.f) {
    r1 = X; g1 = C; b1 = 0.f;
  } else if (H < 180.f) {r1 = 0.f; g1 = C; b1 = X;} else if (H < 240.f) {
    r1 = 0.f; g1 = X; b1 = C;
  } else if (H < 300.f) {r1 = X; g1 = 0.f; b1 = C;} else {r1 = C; g1 = 0.f; b1 = X;}

  R = r1 + m; G = g1 + m; B = b1 + m;
}

inline Ogre::VertexElementType nativeColourType_()
{
  const auto * rs = Ogre::Root::getSingleton().getRenderSystem();
  return rs ? rs->getColourVertexElementType() : Ogre::VET_COLOUR_ARGB;
}

inline Ogre::ColourValue colorFromRainbow(float value, float max_value, float alpha)
{
  if (max_value <= 1e-9f) {
    return Ogre::ColourValue(0.0f, 1.0f, 0.0f, alpha);
  }
  float n = std::max(0.0f, std::min(1.0f, value / max_value));
  const float H = (1.0f - n) * 240.0f;  // blue → red
  float r, g, b;
  hsv2rgb(H, 1.0f, 1.0f, r, g, b);
  return Ogre::ColourValue(r, g, b, alpha);
}

inline Ogre::ColourValue colorFromU8(uint8_t v, float alpha)
{
  if (v == 0) {return Ogre::ColourValue(1.0f, 1.0f, 1.0f, alpha);}
  if (v == 255) {return Ogre::ColourValue(0.25f, 0.25f, 0.25f, alpha);}
  if (v == 254) {return Ogre::ColourValue(0.0f, 0.0f, 0.0f, alpha);}
  float occ = static_cast<float>(v) / 253.0f;
  float c = 1.0f - occ;
  return Ogre::ColourValue(c, c, c, alpha);
}

// Occupancy log-odds (int16, 1/1000): grey 1 - p, as U8 occupancy (free light, occupied black).
inline Ogre::ColourValue colorFromLogodds(int16_t v, float alpha)
{
  const float p = 1.0f / (1.0f + std::exp(-static_cast<float>(v) / 1000.0f));
  return Ogre::ColourValue(1.0f - p, 1.0f - p, 1.0f - p, alpha);
}

inline bool isLogoddsLayer(const navmap_ros_interfaces::msg::NavMapLayer & L)
{
  return L.type == navmap_ros_interfaces::msg::NavMapLayer::I16 && L.unit == "1/1000 log-odds";
}

inline Ogre::ColourValue colorFromHeat(float value, float max_value, float alpha)
{
  if (max_value <= 1e-9f) {return Ogre::ColourValue(0.0f, 1.0f, 0.0f, alpha);}
  float t = std::max(0.0f, std::min(1.0f, value / max_value));
  return Ogre::ColourValue(t, 1.0f - t, 0.0f, alpha);
}

}  // namespace

namespace navmap_rviz_plugin
{

NavMapDisplay::NavMapDisplay()
{
  layer_property_ = new rviz_common::properties::EnumProperty(
    "Layer", "",
    "Select the layer used to colorize triangles.",
    this, SLOT(onLayerSelectionChanged()));

  layer_topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Update Layer Topic", "",
    rosidl_generator_traits::name<navmap_ros_interfaces::msg::NavMapLayer>(),
    "Topic of type navmap_ros_interfaces/NavMapLayer to add/update a layer by name.",
    this, SLOT(updateLayerUpdateTopic()));

  layer_profile_property_ = new rviz_common::properties::QosProfileProperty(
    layer_topic_property_, layer_profile_);

  patch_topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Patch Topic", "",
    rosidl_generator_traits::name<navmap_ros_interfaces::msg::NavMapPatch>(),
    "Topic of type navmap_ros_interfaces/NavMapPatch: changes to the map since the last one, "
    "applied to it (only the changed parts of the mesh are rebuilt). Empty: <topic>_patch.",
    this, SLOT(updatePatchTopic()));

  color_scheme_property_ = new rviz_common::properties::EnumProperty(
    "Color Scheme", "Heat",
    "Color mapping for the active layer. U8: Occupancy. Float: Heat or Rainbow.",
    this, SLOT(onColorSchemeChanged()));

  draw_normals_property_ = new rviz_common::properties::BoolProperty(
    "Draw Normals", false, "Draw one normal per triangle.", this, SLOT(onDrawNormalsChanged()));
  draw_links_property_ = new rviz_common::properties::BoolProperty(
    "Draw Links", true,
    "Draw the links between non-adjacent NavCels (stairs, elevators, doors...), coloured by type.",
    this, SLOT(onDrawLinksChanged()));

  draw_edges_property_ = new rviz_common::properties::BoolProperty(
    "Draw Edges", false,
    "Draw the edges of every NavCel (once each), to see the triangles of the mesh.",
    this, SLOT(onDrawEdgesChanged()));
  edge_color_property_ = new rviz_common::properties::ColorProperty(
    "Edge Color", QColor(255, 255, 255), "Colour of the NavCel edges.", draw_edges_property_,
    SLOT(onDrawEdgesChanged()), this);

  normal_scale_property_ = new rviz_common::properties::FloatProperty(
    "Normal Scale", 0.15f, "Scale of the per-triangle normal arrow/line.",
    this, SLOT(onNormalScaleChanged()));
  normal_scale_property_->setMin(0.0);

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0f, "Triangle color alpha (transparency).", this, SLOT(onAlphaChanged()));
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  info_property_ = new rviz_common::properties::StringProperty(
    "Info", "", "Read-only information about the current NavMap.", this);
  info_property_->setReadOnly(true);
}

NavMapDisplay::~NavMapDisplay()
{
  if (links_obj_) {
    links_obj_->detachFromParent();
    context_->getSceneManager()->destroyManualObject(links_obj_);
    links_obj_ = nullptr;
  }
  if (normals_obj_) {
    normals_obj_->detachFromParent();
    context_->getSceneManager()->destroyManualObject(normals_obj_);
    normals_obj_ = nullptr;
  }
  destroyMesh_();
  if (root_node_) {
    root_node_->removeAndDestroyAllChildren();
    context_->getSceneManager()->destroySceneNode(root_node_);
    root_node_ = nullptr;
  }
}

void NavMapDisplay::onInitialize()
{
  MFDClass::onInitialize();
  layer_topic_property_->initialize(rviz_ros_node_);
  layer_profile_property_->initialize(
    [this](rclcpp::QoS profile) {
      this->layer_profile_ = profile;
      updateLayerUpdateTopic();
    });

  if (!root_node_) {
    root_node_ = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  }

  // Prepare normals object
  if (!links_obj_) {
    links_obj_ = context_->getSceneManager()->createManualObject();
    links_obj_->setDynamic(true);
    root_node_->attachObject(links_obj_);
  }
  if (!normals_obj_) {
    normals_obj_ = context_->getSceneManager()->createManualObject();
    normals_obj_->setDynamic(true);
    root_node_->attachObject(normals_obj_);
  }
}

void NavMapDisplay::reset()
{
  MFDClass::reset();

  last_msg_.reset();
  layers_by_name_.clear();

  layer_property_->clearOptions();
  info_property_->setStdString("");

  navmap_msg_count_ = 0;
  layer_update_count_ = 0;
  patch_count_ = 0;
  last_navmap_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_layer_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  setStatus(rviz_common::properties::StatusProperty::Ok, "NavMap Topic", "waiting for messages…");
  setStatus(rviz_common::properties::StatusProperty::Ok, "Layer Update", "waiting for updates…");

  destroyMesh_();
  updateNormals_();
  updateEdges_();
  updateLinks_();
}

void NavMapDisplay::onEnable()
{
  MFDClass::onEnable();
  subscribeToLayerTopic();
  subscribeToPatchTopic();
}

void NavMapDisplay::onDisable()
{
  unsubscribeToLayerTopic();
  unsubscribeToPatchTopic();
  MFDClass::onDisable();
}

void NavMapDisplay::processMessage(const NavMapMsg::ConstSharedPtr msg)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  geometry_msgs::msg::Pose identity;

  if (!context_->getFrameManager()->transform(
        msg->header, identity, position, orientation))
  {
    setStatus(rviz_common::properties::StatusProperty::Error,
              "TF", "Unable to transform " + QString::fromStdString(msg->header.frame_id));
    return;
  }

  root_node_->setPosition(position);
  root_node_->setOrientation(orientation);

  // A map with other geometry (same number of NavCels or not: a map edited in place keeps it)
  // needs its mesh built again; the same geometry only needs new colours
  const bool same_geometry = last_msg_ &&
    last_msg_->positions_x == msg->positions_x && last_msg_->positions_y == msg->positions_y &&
    last_msg_->positions_z == msg->positions_z && last_msg_->navcels_v0 == msg->navcels_v0 &&
    last_msg_->navcels_v1 == msg->navcels_v1 && last_msg_->navcels_v2 == msg->navcels_v2;
  if (!same_geometry) {destroyMesh_();}

  last_msg_ = std::make_shared<NavMapMsg>(*msg);
  received_msg = last_msg_;   // the tools build their NavMap from it when they need it
  received_stale = true;

  ++navmap_msg_count_;
  last_navmap_stamp_ = rviz_ros_node_.lock()->get_raw_node()->now();
  {
    QString s = QString("OK — msgs: %1 | last: %2.%3 s")
      .arg(qulonglong(navmap_msg_count_))
      .arg(last_navmap_stamp_.seconds())
      .arg(last_navmap_stamp_.nanoseconds() % 1000000000, 9, 10, QChar('0'));
    setStatus(rviz_common::properties::StatusProperty::Ok, "NavMap Topic", s);
  }

  {
    std::string info = "Surfaces: " + std::to_string(last_msg_->surfaces.size()) +
      " | Triangles: " + std::to_string(last_msg_->navcels_v0.size());
    info_property_->setStdString(info);
  }

  repopulateLayerEnum_();
  rebuildLayerIndex_();
  updateColorSchemeOptions_();

  if (layer_topic_property_->isEmpty() && !topic_property_->isEmpty()) {
    layer_topic_property_->setStdString(topic_property_->getTopicStd() + "_layers");
    updateLayerUpdateTopic();
  }
  if (patch_topic_property_->isEmpty() && !topic_property_->isEmpty()) {
    patch_topic_property_->setStdString(topic_property_->getTopicStd() + "_patch");
    updatePatchTopic();
  }

  // Build/refresh mesh and color buffer
  ensureMeshBuilt_();
  updateColorsOnly_();
  updateNormals_();
  if (!same_geometry) {updateEdges_();}
  updateLinks_();

  context_->queueRender();
}

void NavMapDisplay::subscribeToLayerTopic()
{
  if (!isEnabled()) {
    return;
  }
  if (layer_topic_property_->isEmpty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Layer Update Topic", "Empty topic name (layer updates disabled)");
    return;
  }

  try {
    auto node = rviz_ros_node_.lock()->get_raw_node();

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.event_callbacks.message_lost_callback =
      [this](rclcpp::QOSMessageLostInfo & info)
      {
        std::ostringstream s;
        s << "Some layer messages were lost. New lost: "
          << info.total_count_change << " | Total lost: "
          << info.total_count;
        setStatus(rviz_common::properties::StatusProperty::Warn, "Layer Update Topic",
          s.str().c_str());
      };

    layer_subscription_ =
      node->create_subscription<NavMapLayerMsg>(
        layer_topic_property_->getTopicStd(),
        layer_profile_,
      [this](NavMapLayerMsg::ConstSharedPtr msg) {incomingLayer(msg);},
        sub_opts);

    layer_subscription_start_time_ = node->now();
    setStatus(rviz_common::properties::StatusProperty::Ok, "Layer Update Topic", "OK");
  } catch (const rclcpp::exceptions::InvalidTopicNameError & e) {
    setStatus(rviz_common::properties::StatusProperty::Error,
      "Layer Update Topic", QString("Invalid topic: ") + e.what());
  } catch (const std::exception & e) {
    setStatus(rviz_common::properties::StatusProperty::Error,
      "Layer Update Topic", QString("Failed to subscribe: ") + e.what());
  }
}

void NavMapDisplay::unsubscribeToLayerTopic()
{
  layer_subscription_.reset();
}

void NavMapDisplay::updateLayerUpdateTopic()
{
  unsubscribeToLayerTopic();
  subscribeToLayerTopic();
  context_->queueRender();
}

void NavMapDisplay::subscribeToPatchTopic()
{
  if (!isEnabled() || patch_topic_property_->isEmpty()) {return;}
  try {
    auto node = rviz_ros_node_.lock()->get_raw_node();
    patch_subscription_ = node->create_subscription<NavMapPatchMsg>(
      patch_topic_property_->getTopicStd(), rclcpp::QoS(10).reliable(),
      [this](NavMapPatchMsg::ConstSharedPtr msg) {incomingPatch(msg);});
    setStatus(rviz_common::properties::StatusProperty::Ok, "Patch Topic", "OK");
  } catch (const std::exception & e) {
    setStatus(rviz_common::properties::StatusProperty::Error,
      "Patch Topic", QString("Failed to subscribe: ") + e.what());
  }
}

void NavMapDisplay::unsubscribeToPatchTopic()
{
  patch_subscription_.reset();
}

void NavMapDisplay::updatePatchTopic()
{
  unsubscribeToPatchTopic();
  subscribeToPatchTopic();
  context_->queueRender();
}

// Changes to the map shown: applied to it, and only the chunks of the mesh they touch are built
// again. A patch for another map is ignored until the next whole map arrives.
void NavMapDisplay::incomingPatch(const NavMapPatchMsg::ConstSharedPtr & msg)
{
  if (!last_msg_) {
    setStatus(rviz_common::properties::StatusProperty::Warn, "Patch Topic",
      "Patch before any NavMap: waiting for a whole map");
    return;
  }
  const std::size_t old_cels = last_msg_->navcels_v0.size();
  const std::size_t old_layers = last_msg_->layers.size();
  if (!navmap_ros::apply_patch(*msg, *last_msg_)) {
    setStatus(rviz_common::properties::StatusProperty::Warn, "Patch Topic",
      "A patch for another map: waiting for the next whole map");
    return;
  }
  received_stale = true;
  rebuildLayerIndex_();
  if (last_msg_->layers.size() != old_layers) {
    repopulateLayerEnum_();
    updateColorSchemeOptions_();
  }

  // Chunks touched: those of the rewritten NavCels, and the new ones at the end
  const std::size_t n = last_msg_->navcels_v0.size();
  const std::size_t nchunks = (n + kChunkCels - 1) / kChunkCels;
  std::vector<char> touched(nchunks, 0);
  for (const auto c : msg->navcel_indices) {
    if (c < n) {touched[c / kChunkCels] = 1;}
  }
  if (n != old_cels) {
    for (std::size_t k = std::min(old_cels, n) / kChunkCels; k < nchunks; ++k) {
      touched[k] = 1;
    }
  }
  while (chunks_.size() > nchunks) {
    destroyChunk_(chunks_.back());
    chunks_.pop_back();
  }
  chunks_.resize(nchunks);
  const auto ctx = colourContext_();
  std::size_t rebuilt = 0;
  for (std::size_t k = 0; k < nchunks; ++k) {
    if (!touched[k] && chunks_[k].entity) {continue;}
    buildChunk_(k);
    colourChunk_(k, ctx);
    edgesChunk_(k);
    ++rebuilt;
  }
  if (draw_normals_property_->getBool()) {updateNormals_();}
  updateLinks_();

  ++patch_count_;
  info_property_->setStdString(
    "Surfaces: " + std::to_string(last_msg_->surfaces.size()) +
    " | Triangles: " + std::to_string(n));
  setStatus(rviz_common::properties::StatusProperty::Ok, "Patch Topic",
    QString("OK — patches: %1 | last: %2 NavCels, %3 of %4 chunks rebuilt")
    .arg(qulonglong(patch_count_)).arg(qulonglong(msg->navcel_indices.size()))
    .arg(qulonglong(rebuilt)).arg(qulonglong(nchunks)));
  context_->queueRender();
}

void NavMapDisplay::incomingLayer(const NavMapLayerMsg::ConstSharedPtr & msg)
{
  if (!last_msg_) {
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Layer Update",
      "Received a layer before any NavMap; it will be applied upon the next NavMap message.");
    return;
  }

  if (msg->geometry_id != 0 && last_msg_->geometry_id != 0 &&
    msg->geometry_id != last_msg_->geometry_id)
  {
    setStatus(rviz_common::properties::StatusProperty::Warn, "Layer Update",
      QString("Ignored \"%1\": it belongs to another geometry").arg(
        QString::fromStdString(msg->name)));
    return;
  }

  // Sparse update: patch the cached layer of the same name and type
  if (!msg->indices.empty()) {
    NavMapLayerMsg * cached = nullptr;
    for (auto & L : last_msg_->layers) {
      if (L.name == msg->name && L.type == msg->type) {cached = &L;}
    }
    auto patch = [&](auto & dst, const auto & src) {
        if (src.size() != msg->indices.size()) {return false;}
        for (size_t k = 0; k < msg->indices.size(); ++k) {
          if (msg->indices[k] < dst.size()) {dst[msg->indices[k]] = src[k];}
        }
        return true;
      };
    bool ok = cached != nullptr;
    if (ok) {
      switch (msg->type) {
        case NavMapLayerMsg::U8: ok = patch(cached->data_u8, msg->data_u8); break;
        case NavMapLayerMsg::F32: ok = patch(cached->data_f32, msg->data_f32); break;
        case NavMapLayerMsg::F64: ok = patch(cached->data_f64, msg->data_f64); break;
        case NavMapLayerMsg::I16: ok = patch(cached->data_i16, msg->data_i16); break;
        default: ok = false;
      }
    }
    if (!ok) {
      setStatus(rviz_common::properties::StatusProperty::Error, "Layer Update",
        QString("Sparse update of \"%1\" does not match a received layer").arg(
          QString::fromStdString(msg->name)));
      return;
    }
    ++layer_update_count_;
    if (currentSelectedLayer_() == msg->name) {updateColorsOnly_();}
    setStatus(rviz_common::properties::StatusProperty::Ok, "Layer Update",
      QString("OK — updates: %1 | last: \"%2\" (sparse, %3 cells)")
      .arg(qulonglong(layer_update_count_)).arg(QString::fromStdString(msg->name))
      .arg(qulonglong(msg->indices.size())));
    context_->queueRender();
    return;
  }

  const size_t n_tris = last_msg_->navcels_v0.size();
  const size_t n_u8 = msg->data_u8.size();
  const size_t n_f32 = msg->data_f32.size();
  const size_t n_f64 = msg->data_f64.size();
  const size_t n_i16 = msg->data_i16.size();
  const int non_empty = (n_u8 ? 1 : 0) + (n_f32 ? 1 : 0) + (n_f64 ? 1 : 0) + (n_i16 ? 1 : 0);

  if (non_empty != 1) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Layer Update",
      "Exactly one of data_u8 / data_f32 / data_f64 / data_i16 must be non-empty.");
    return;
  }
  const size_t eff_len = n_u8 ? n_u8 : (n_f32 ? n_f32 : (n_f64 ? n_f64 : n_i16));
  if (eff_len != n_tris) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Layer Update",
      QString("Layer size (%1) does not match number of triangles (%2)")
      .arg(eff_len).arg(n_tris));
    return;
  }

  applyOrCacheLayer_(*msg);

  ++layer_update_count_;
  last_layer_stamp_ = rviz_ros_node_.lock()->get_raw_node()->now();

  const char * type_str =
    (msg->type == navmap_ros_interfaces::msg::NavMapLayer::U8) ? "U8" :
    (msg->type == navmap_ros_interfaces::msg::NavMapLayer::F32) ? "F32" :
    (msg->type == navmap_ros_interfaces::msg::NavMapLayer::F64) ? "F64" :
    (msg->type == navmap_ros_interfaces::msg::NavMapLayer::I16) ? "I16" : "UNKNOWN";

  const size_t len = eff_len;

  QString line = QString("OK — updates: %1 | last: \"%2\" (%3, len=%4)")
    .arg(qulonglong(layer_update_count_))
    .arg(QString::fromStdString(msg->name))
    .arg(type_str)
    .arg(qulonglong(len));
  setStatus(rviz_common::properties::StatusProperty::Ok,
            "Layer Update Topic", line);

  if (currentSelectedLayer_() == msg->name) {
    updateColorSchemeOptions_();
    // Fast path: only recolor, geometry unchanged
    updateColorsOnly_();
  }
  if (draw_normals_property_->getBool()) {
    updateNormals_();
  }

  setStatus(rviz_common::properties::StatusProperty::Ok, "Layer Update", "OK");
  context_->queueRender();
}

void NavMapDisplay::rebuildLayerIndex_()
{
  layers_by_name_.clear();
  if (!last_msg_) {return;}
  for (const auto & L : last_msg_->layers) {
    layers_by_name_.emplace(L.name, &L);
  }
}

std::string NavMapDisplay::currentSelectedLayer_() const
{
  const QString q = layer_property_->getString();
  return q.isEmpty() ? std::string() : q.toStdString();
}

void NavMapDisplay::repopulateLayerEnum_()
{
  const std::string prev = currentSelectedLayer_();

  layer_property_->clearOptions();
  if (last_msg_) {
    for (const auto & L : last_msg_->layers) {
      layer_property_->addOption(QString::fromStdString(L.name));
    }
    if (last_msg_->has_vertex_rgba &&
      last_msg_->colors_r.size() == last_msg_->positions_x.size() &&
      last_msg_->colors_g.size() == last_msg_->positions_x.size() &&
      last_msg_->colors_b.size() == last_msg_->positions_x.size())
    {
      layer_property_->addOption(kVertexColorLayerName);
    }
  }
  if (!prev.empty()) {
    layer_property_->setString(prev.c_str());
  }
}

void NavMapDisplay::applyOrCacheLayer_(const NavMapLayerMsg & layer)
{
  if (!last_msg_) {return;}

  bool found = false;
  for (auto & L : last_msg_->layers) {
    if (L.name == layer.name) {
      L = layer;
      found = true;
      break;
    }
  }
  if (!found) {
    last_msg_->layers.push_back(layer);
    layer_property_->addOption(QString::fromStdString(layer.name));
  }
  rebuildLayerIndex_();
}

void NavMapDisplay::onLayerSelectionChanged()
{
  updateColorSchemeOptions_();
  // Fast recolor on same geometry
  updateColorsOnly_();
  if (draw_normals_property_->getBool()) {
    updateNormals_();
  }
  context_->queueRender();
}

void NavMapDisplay::onDrawNormalsChanged()
{
  updateNormals_();
  context_->queueRender();
}

void NavMapDisplay::onAlphaChanged()
{
  // Alpha affects only colors, not geometry
  updateColorsOnly_();
  context_->queueRender();
}

void NavMapDisplay::onNormalScaleChanged()
{
  if (draw_normals_property_->getBool()) {
    updateNormals_();
    context_->queueRender();
  }
}

void NavMapDisplay::onColorSchemeChanged()
{
  updateColorsOnly_();
  if (draw_normals_property_->getBool()) {
    updateNormals_();
  }
  context_->queueRender();
}

// ----- Mesh in chunks: built once, recoloured fast; a patch rebuilds only its chunks -----

void NavMapDisplay::ensureMeshBuilt_()
{
  if (!context_ || !last_msg_) {return;}
  const std::size_t n = last_msg_->navcels_v0.size();
  if (n == 0 || last_msg_->positions_x.empty()) {return;}
  const std::size_t nchunks = (n + kChunkCels - 1) / kChunkCels;
  while (chunks_.size() > nchunks) {
    destroyChunk_(chunks_.back());
    chunks_.pop_back();
  }
  chunks_.resize(nchunks);
  for (std::size_t k = 0; k < nchunks; ++k) {
    if (!chunks_[k].entity) {buildChunk_(k);}
  }
}

void NavMapDisplay::buildChunk_(std::size_t k)
{
  if (!context_ || !last_msg_ || k >= chunks_.size()) {return;}
  auto * sm = context_->getSceneManager();
  Chunk & ch = chunks_[k];
  // the geometry again; the edges are rebuilt apart (edgesChunk_)
  if (ch.entity) {
    ch.entity->detachFromParent();
    sm->destroyEntity(ch.entity);
    ch.entity = nullptr;
  }
  if (ch.mesh) {
    Ogre::MeshManager::getSingleton().remove(ch.mesh->getHandle());
    ch.mesh.reset();
  }
  ch.colour.reset();

  const auto & X = last_msg_->positions_x;
  const auto & Y = last_msg_->positions_y;
  const auto & Z = last_msg_->positions_z;
  const auto & V0 = last_msg_->navcels_v0;
  const auto & V1 = last_msg_->navcels_v1;
  const auto & V2 = last_msg_->navcels_v2;
  const std::size_t first = k * kChunkCels;
  const std::size_t last = std::min(V0.size(), first + kChunkCels);
  if (first >= last) {return;}
  const std::size_t nv = X.size();

  // 1) Positions (one vertex per corner: colours are per NavCel), with a neutral colour
  Ogre::ManualObject * mo = sm->createManualObject();
  mo->setDynamic(false);
  mo->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
  const Ogre::ColourValue neutral(0.7f, 0.7f, 0.7f, alpha_property_->getFloat());
  for (std::size_t t = first; t < last; ++t) {
    for (const uint32_t i : {V0[t], V1[t], V2[t]}) {
      if (i < nv) {mo->position(X[i], Y[i], Z[i]);} else {mo->position(0.0f, 0.0f, 0.0f);}
      mo->colour(neutral);
    }
  }
  mo->end();

  static int mesh_counter = 0;
  const std::string mesh_name = "NavMapMesh_" + std::to_string(++mesh_counter);
  ch.mesh = mo->convertToMesh(mesh_name);
  mo->detachFromParent();
  sm->destroyManualObject(mo);

  // Material with vertex-colour tracking, shared by the chunks
  if (material_.empty()) {
    Ogre::MaterialPtr base =
      Ogre::MaterialManager::getSingleton().getByName("BaseWhiteNoLighting");
    Ogre::MaterialPtr mat = base->clone(mesh_name + "/mat");
    mat->setLightingEnabled(false);
    mat->getTechnique(0)->getPass(0)->setVertexColourTracking(
      Ogre::TVC_AMBIENT | Ogre::TVC_DIFFUSE);
    material_ = mat->getName();
  }

  ch.entity = sm->createEntity(mesh_name);
  ch.entity->setMaterialName(material_);
  if (!root_node_) {
    root_node_ = sm->getRootSceneNode()->createChildSceneNode();
  }
  root_node_->attachObject(ch.entity);

  // 2) The colour in its own dynamic vertex buffer, rewritten without touching the positions
  Ogre::SubMesh * sub = ch.mesh->getSubMesh(0);
  Ogre::VertexData * vdata = sub->useSharedVertices ? ch.mesh->sharedVertexData : sub->vertexData;
  Ogre::VertexDeclaration * decl = vdata->vertexDeclaration;
  Ogre::VertexBufferBinding * bind = vdata->vertexBufferBinding;
  if (decl->findElementBySemantic(Ogre::VES_DIFFUSE)) {
    decl->removeElement(Ogre::VES_DIFFUSE);
  }
  uint16_t src = 1;
  while (bind->isBufferBound(src)) {++src;}
  // the render system's own packing (ABGR with OpenGL, ARGB with D3D): a fixed ARGB swaps red and
  // blue where it is not the native one
  const Ogre::VertexElementType col_type = nativeColourType_();
  decl->addElement(src, 0, col_type, Ogre::VES_DIFFUSE);
  ch.colour = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
    Ogre::VertexElement::getTypeSize(col_type), vdata->vertexCount,
    Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
  bind->setBinding(src, ch.colour);
}

void NavMapDisplay::destroyChunk_(Chunk & ch)
{
  if (!context_) {return;}
  auto * sm = context_->getSceneManager();
  if (ch.entity) {
    ch.entity->detachFromParent();
    sm->destroyEntity(ch.entity);
    ch.entity = nullptr;
  }
  if (ch.mesh) {
    Ogre::MeshManager::getSingleton().remove(ch.mesh->getHandle());
    ch.mesh.reset();
  }
  ch.colour.reset();
  if (ch.edges) {
    ch.edges->detachFromParent();
    sm->destroyManualObject(ch.edges);
    ch.edges = nullptr;
  }
}

void NavMapDisplay::destroyMesh_()
{
  for (auto & ch : chunks_) {
    destroyChunk_(ch);
  }
  chunks_.clear();
}

NavMapDisplay::ColourCtx NavMapDisplay::colourContext_() const
{
  ColourCtx ctx;
  if (!last_msg_) {return ctx;}
  const std::size_t n = last_msg_->navcels_v0.size();
  ctx.alpha = alpha_property_->getFloat();
  ctx.rainbow = (color_scheme_property_->getStdString() == "Rainbow");
  const std::string sel = currentSelectedLayer_();
  auto it = layers_by_name_.find(sel);
  if (it != layers_by_name_.end()) {
    ctx.layer = it->second;
  } else if (sel == kVertexColorLayerName) {
    ctx.vertex_colours = last_msg_->has_vertex_rgba &&
      last_msg_->colors_r.size() == last_msg_->positions_x.size() &&
      last_msg_->colors_g.size() == last_msg_->positions_x.size() &&
      last_msg_->colors_b.size() == last_msg_->positions_x.size();
  }
  if (ctx.layer) {
    const auto & L = *ctx.layer;
    if (L.type == NavMapLayerMsg::U8 && L.data_u8.size() == n) {
      ctx.is_u8 = true;
    } else if (L.type == NavMapLayerMsg::I16 && L.data_i16.size() == n) {
      if (!L.data_i16.empty()) {
        const auto [lo, hi] = std::minmax_element(L.data_i16.begin(), L.data_i16.end());
        ctx.min_val = *lo;
        ctx.max_val = *hi;
      }
      ctx.logodds = isLogoddsLayer(L);
    } else if (L.type == NavMapLayerMsg::F32 && L.data_f32.size() == n) {
      for (float v : L.data_f32) {
        ctx.max_val = std::max(ctx.max_val, v);
      }
    } else if (L.type == NavMapLayerMsg::F64 && L.data_f64.size() == n) {
      for (double v : L.data_f64) {
        ctx.max_val = std::max(ctx.max_val, static_cast<float>(v));
      }
    } else {
      ctx.layer = nullptr;  // neutral grey
    }
  }
  return ctx;
}

void NavMapDisplay::colourChunk_(std::size_t k, const ColourCtx & ctx)
{
  if (!last_msg_ || k >= chunks_.size() || !chunks_[k].colour) {return;}
  const auto & V0 = last_msg_->navcels_v0;
  const auto & V1 = last_msg_->navcels_v1;
  const auto & V2 = last_msg_->navcels_v2;
  const std::size_t first = k * kChunkCels;
  const std::size_t last = std::min(V0.size(), first + kChunkCels);
  if (first >= last || chunks_[k].colour->getNumVertices() != 3 * (last - first)) {return;}

  auto pack = [](const Ogre::ColourValue & c) -> uint32_t {
      // the render system's colour format, the one the buffer was created with
      return Ogre::VertexElement::convertColourValue(c, nativeColourType_());
    };
  const float alpha = ctx.alpha;
  const auto * L = ctx.layer;
  auto colour_of = [&](std::size_t t) -> Ogre::ColourValue {
      if (L->type == NavMapLayerMsg::U8) {return colorFromU8(L->data_u8[t], alpha);}
      if (L->type == NavMapLayerMsg::I16) {
        const int16_t v = L->data_i16[t];
        const float shifted = static_cast<float>(v) - ctx.min_val;  // the scales start at 0
        return ctx.logodds ? colorFromLogodds(v, alpha) :
               (ctx.rainbow ? colorFromRainbow(shifted, ctx.max_val - ctx.min_val, alpha) :
               colorFromHeat(shifted, ctx.max_val - ctx.min_val, alpha));
      }
      const float v = L->type == NavMapLayerMsg::F32 ? L->data_f32[t] :
        static_cast<float>(L->data_f64[t]);
      return ctx.rainbow ? colorFromRainbow(v, ctx.max_val, alpha) :
             colorFromHeat(v, ctx.max_val, alpha);
    };
  // Cells holding the layer's unknown value: translucent grey (U8 keeps its own mapping)
  auto unknown = [&](std::size_t t) -> bool {
      if (ctx.is_u8 || !L->has_unknown) {return false;}
      const double u = L->unknown_value;
      const double v = L->type == NavMapLayerMsg::I16 ? L->data_i16[t] :
        L->type == NavMapLayerMsg::F32 ? L->data_f32[t] : L->data_f64[t];
      return (std::isnan(u) && std::isnan(v)) || v == u;
    };
  const uint32_t grey = pack(Ogre::ColourValue(0.5f, 0.5f, 0.5f, 0.35f * alpha));
  const uint32_t neutral = pack(Ogre::ColourValue(0.7f, 0.7f, 0.7f, alpha));

  auto * p = static_cast<uint32_t *>(chunks_[k].colour->lock(Ogre::HardwareBuffer::HBL_DISCARD));
  const std::size_t nv = last_msg_->positions_x.size();
  for (std::size_t t = first; t < last; ++t) {
    if (ctx.vertex_colours) {
      const auto & R = last_msg_->colors_r;
      const auto & G = last_msg_->colors_g;
      const auto & B = last_msg_->colors_b;
      const auto & A = last_msg_->colors_a;
      for (const uint32_t i : {V0[t], V1[t], V2[t]}) {
        if (i >= nv) {*p++ = neutral; continue;}
        *p++ = pack(
          Ogre::ColourValue(
            R[i] / 255.f, G[i] / 255.f, B[i] / 255.f,
            A.size() == nv ? (A[i] / 255.f) * alpha : alpha));
      }
      continue;
    }
    const uint32_t c = !L ? neutral : (unknown(t) ? grey : pack(colour_of(t)));
    *p++ = c; *p++ = c; *p++ = c;
  }
  chunks_[k].colour->unlock();
}

void NavMapDisplay::updateColorsOnly_()
{
  ensureMeshBuilt_();
  const auto ctx = colourContext_();
  for (std::size_t k = 0; k < chunks_.size(); ++k) {
    colourChunk_(k, ctx);
  }
}

// The edges of the chunk's NavCels, each once within the chunk, lifted 2 cm so they do not
// flicker against the surface. Degenerate NavCels (placeholders) are left out.
void NavMapDisplay::edgesChunk_(std::size_t k)
{
  if (!context_ || k >= chunks_.size()) {return;}
  Chunk & ch = chunks_[k];
  if (!last_msg_ || !draw_edges_property_->getBool()) {
    if (ch.edges) {ch.edges->clear();}
    return;
  }
  if (!ch.edges) {
    ch.edges = context_->getSceneManager()->createManualObject();
    ch.edges->setDynamic(true);
    root_node_->attachObject(ch.edges);
  }
  ch.edges->clear();

  const auto & X = last_msg_->positions_x;
  const auto & Y = last_msg_->positions_y;
  const auto & Z = last_msg_->positions_z;
  const auto & V0 = last_msg_->navcels_v0;
  const auto & V1 = last_msg_->navcels_v1;
  const auto & V2 = last_msg_->navcels_v2;
  const std::size_t first = k * kChunkCels;
  const std::size_t last = std::min(V0.size(), first + kChunkCels);
  const std::size_t nv = X.size();
  std::vector<std::uint64_t> edges;
  edges.reserve(3 * (last > first ? last - first : 0));
  for (std::size_t t = first; t < last; ++t) {
    const uint32_t v[3] = {V0[t], V1[t], V2[t]};
    if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2]) {continue;}
    if (v[0] >= nv || v[1] >= nv || v[2] >= nv) {continue;}
    for (int e = 0; e < 3; ++e) {
      const uint32_t a = std::min(v[e], v[(e + 1) % 3]), b = std::max(v[e], v[(e + 1) % 3]);
      edges.push_back((static_cast<std::uint64_t>(a) << 32) | b);
    }
  }
  std::sort(edges.begin(), edges.end());
  edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
  if (edges.empty()) {return;}

  const QColor q = edge_color_property_->getColor();
  const Ogre::ColourValue colour(q.redF(), q.greenF(), q.blueF(), 1.0f);
  const Ogre::Vector3 lift(0.0f, 0.0f, 0.02f);
  ch.edges->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
  for (const auto key : edges) {
    const auto a = static_cast<uint32_t>(key >> 32), b = static_cast<uint32_t>(key & 0xffffffffu);
    ch.edges->position(Ogre::Vector3(X[a], Y[a], Z[a]) + lift);
    ch.edges->colour(colour);
    ch.edges->position(Ogre::Vector3(X[b], Y[b], Z[b]) + lift);
    ch.edges->colour(colour);
  }
  ch.edges->end();
}

void NavMapDisplay::updateGeometry_()
{
  // For Option B, geometry build is isolated; colors are updated separately.
  ensureMeshBuilt_();
  updateColorsOnly_();
}

void NavMapDisplay::updateNormals_()
{
  if (!context_ || !last_msg_ || !normals_obj_) {return;}

  normals_obj_->clear();
  if (!draw_normals_property_->getBool()) {return;}

  const auto & X = last_msg_->positions_x;
  const auto & Y = last_msg_->positions_y;
  const auto & Z = last_msg_->positions_z;
  const auto & V0 = last_msg_->navcels_v0;
  const auto & V1 = last_msg_->navcels_v1;
  const auto & V2 = last_msg_->navcels_v2;

  if (X.empty() || V0.empty()) {return;}

  const float len = std::max(0.0f, normal_scale_property_->getFloat());

  normals_obj_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
  for (size_t t = 0; t < V0.size(); ++t) {
    const uint32_t i0 = V0[t], i1 = V1[t], i2 = V2[t];
    const Ogre::Vector3 p0(X[i0], Y[i0], Z[i0]);
    const Ogre::Vector3 p1(X[i1], Y[i1], Z[i1]);
    const Ogre::Vector3 p2(X[i2], Y[i2], Z[i2]);

    const Ogre::Vector3 c = (p0 + p1 + p2) / 3.0f;
    Ogre::Vector3 n = (p1 - p0).crossProduct(p2 - p0);
    if (n.squaredLength() > 1e-12f) {n.normalise();}

    const Ogre::Vector3 tip = c + len * n;

    normals_obj_->position(c);
    normals_obj_->colour(Ogre::ColourValue(0, 0, 1, 1));
    normals_obj_->position(tip);
    normals_obj_->colour(Ogre::ColourValue(0, 0, 1, 1));
  }
  normals_obj_->end();
}

void NavMapDisplay::onDrawEdgesChanged()
{
  updateEdges_();
  context_->queueRender();
}

void NavMapDisplay::updateEdges_()
{
  for (std::size_t k = 0; k < chunks_.size(); ++k) {
    edgesChunk_(k);
  }
}

void NavMapDisplay::onDrawLinksChanged()
{
  updateLinks_();
  context_->queueRender();
}

// Links between non-adjacent NavCels: a line between their centroids, raised in the middle so a
// link between stacked NavCels stays visible, coloured by type.
void NavMapDisplay::updateLinks_()
{
  if (!context_ || !last_msg_ || !links_obj_) {return;}
  links_obj_->clear();
  if (!draw_links_property_->getBool() || last_msg_->links.empty()) {return;}

  const auto & X = last_msg_->positions_x;
  const auto & Y = last_msg_->positions_y;
  const auto & Z = last_msg_->positions_z;
  const auto & V0 = last_msg_->navcels_v0;
  const auto & V1 = last_msg_->navcels_v1;
  const auto & V2 = last_msg_->navcels_v2;
  auto centroid = [&](uint32_t t) {
      return Ogre::Vector3(
        (X[V0[t]] + X[V1[t]] + X[V2[t]]) / 3.0f, (Y[V0[t]] + Y[V1[t]] + Y[V2[t]]) / 3.0f,
        (Z[V0[t]] + Z[V1[t]] + Z[V2[t]]) / 3.0f);
    };
  auto colour = [](uint8_t type) {
      switch (type) {
        case navmap_ros_interfaces::msg::NavMapLink::RAMP: return Ogre::ColourValue(0, 0.8f, 0, 1);
        case navmap_ros_interfaces::msg::NavMapLink::STAIRS: return Ogre::ColourValue(1, 0.5f, 0,
          1);
        case navmap_ros_interfaces::msg::NavMapLink::ELEVATOR:
          return Ogre::ColourValue(0.6f, 0, 1, 1);
        case navmap_ros_interfaces::msg::NavMapLink::DOOR: return Ogre::ColourValue(0, 0.6f, 1, 1);
        default: return Ogre::ColourValue(1, 1, 0, 1);
      }
    };

  links_obj_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
  for (const auto & l : last_msg_->links) {
    if (l.from_navcel >= V0.size() || l.to_navcel >= V0.size()) {continue;}
    const Ogre::Vector3 a = centroid(l.from_navcel), b = centroid(l.to_navcel);
    const Ogre::Vector3 mid = 0.5f * (a + b) + Ogre::Vector3(0, 0, 0.25f * (a - b).length());
    const Ogre::ColourValue c = colour(l.type);
    links_obj_->position(a); links_obj_->colour(c);
    links_obj_->position(mid); links_obj_->colour(c);
    links_obj_->position(mid); links_obj_->colour(c);
    links_obj_->position(b); links_obj_->colour(c);
  }
  links_obj_->end();
}

void NavMapDisplay::updateColorSchemeOptions_()
{
  const QString prev = color_scheme_property_->getString();
  color_scheme_property_->clearOptions();

  const std::string sel = currentSelectedLayer_();
  auto it = layers_by_name_.find(sel);

  if (it != layers_by_name_.end()) {
    const auto * L = it->second;
    if (L->type == navmap_ros_interfaces::msg::NavMapLayer::U8 || isLogoddsLayer(*L)) {
      color_scheme_property_->addOption("Occupancy");
      color_scheme_property_->setString("Occupancy");
      color_scheme_property_->setDescription(
        "U8 occupancy mapping (free=light, occ=black, unknown=dark green).");
      return;
    } else {
      color_scheme_property_->addOption("Heat");
      color_scheme_property_->addOption("Rainbow");
      if (prev == "Rainbow") {color_scheme_property_->setString("Rainbow");} else {
        color_scheme_property_->setString("Heat");
      }
      color_scheme_property_->setDescription(
        "Float mapping: Heat (red/yellow) or Rainbow (HSV spectrum).");
      return;
    }
  }

  // Fallback
  color_scheme_property_->addOption("Heat");
  color_scheme_property_->addOption("Rainbow");
  if (prev == "Rainbow") {color_scheme_property_->setString("Rainbow");} else {
    color_scheme_property_->setString("Heat");
  }
}

}  // namespace navmap_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(navmap_rviz_plugin::NavMapDisplay, rviz_common::Display)
