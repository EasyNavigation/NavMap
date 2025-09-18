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


#include "navmap_rviz_plugin/NavMapDisplay.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreManualObject.h>

#include <rclcpp/exceptions.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/uniform_string_stream.hpp>
#include <rviz_common/validate_floats.hpp>

#include <QCoreApplication>

namespace
{

// Occupancy-style U8 mapping with global alpha.
// 0 -> light gray (free), 254 -> black (occupied), 255 -> dark green (unknown), 1..253 -> inverted gray.
inline Ogre::ColourValue colorFromU8(uint8_t v, float alpha)
{
  if (v == 0) {return Ogre::ColourValue(0.5f, 0.5f, 0.5f, alpha);}
  if (v == 255) {return Ogre::ColourValue(0.0f, 0.39f, 0.0f, alpha);}
  if (v == 254) {return Ogre::ColourValue(0.0f, 0.0f, 0.0f, alpha);}
  float occ = static_cast<float>(v) / 253.0f;
  float c = 1.0f - occ;
  return Ogre::ColourValue(c, c, c, alpha);
}

// Simple blue→red heatmap; if max is ~0, fall back to solid green.
inline Ogre::ColourValue colorFromHeat(float value, float max_value, float alpha)
{
  if (max_value <= 1e-9f) {return Ogre::ColourValue(0.0f, 1.0f, 0.0f, alpha);}
  float t = std::max(0.0f, std::min(1.0f, value / max_value));
  float r = t;
  float g = 1.0f - t;
  float b = 0.0f;
  return Ogre::ColourValue(r, g, b, alpha);
}

} // namespace

namespace navmap_rviz_plugin
{

NavMapDisplay::NavMapDisplay()
{
  // Properties
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

  draw_normals_property_ = new rviz_common::properties::BoolProperty(
    "Draw Normals", false, "Draw one normal per triangle.", this, SLOT(onDrawNormalsChanged()));

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
  // Destroy OGRE objects if they exist
  if (triangles_obj_) {
    triangles_obj_->detachFromParent();
    context_->getSceneManager()->destroyManualObject(triangles_obj_);
    triangles_obj_ = nullptr;
  }
  if (normals_obj_) {
    normals_obj_->detachFromParent();
    context_->getSceneManager()->destroyManualObject(normals_obj_);
    normals_obj_ = nullptr;
  }
  if (root_node_) {
    root_node_->removeAndDestroyAllChildren();
    context_->getSceneManager()->destroySceneNode(root_node_);
    root_node_ = nullptr;
  }
}

void NavMapDisplay::onInitialize()
{
  MFDClass::onInitialize();

  // Initialize QoS-configurable layer subscription
  layer_topic_property_->initialize(rviz_ros_node_);
  layer_profile_property_->initialize(
    [this](rclcpp::QoS profile) {
      this->layer_profile_ = profile;
      updateLayerUpdateTopic();
    });

  // Create root scene node if needed
  if (!root_node_) {
    root_node_ = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  }
}

void NavMapDisplay::reset()
{
  MFDClass::reset();

  last_msg_.reset();
  layers_by_name_.clear();

  layer_property_->clearOptions();
  info_property_->setStdString("");

  updateGeometry_();
  updateNormals_();
}

void NavMapDisplay::onEnable()
{
  MFDClass::onEnable();
  subscribeToLayerTopic();
}

void NavMapDisplay::onDisable()
{
  unsubscribeToLayerTopic();
  MFDClass::onDisable();
}

void NavMapDisplay::processMessage(const NavMapMsg::ConstSharedPtr msg)
{
  // Keep a mutable copy to integrate layer updates
  last_msg_ = std::make_shared<NavMapMsg>(*msg);

  {
    std::string info = "Surfaces: " + std::to_string(last_msg_->surfaces.size()) +
      " | Triangles: " + std::to_string(last_msg_->navcels_v0.size());
    info_property_->setStdString(info);
  }

  repopulateLayerEnum_();
  rebuildLayerIndex_();

  // Autocomplete a default layer topic from the base topic (optional)
  if (layer_topic_property_->isEmpty() && !topic_property_->isEmpty()) {
    layer_topic_property_->setStdString(topic_property_->getTopicStd() + "_layers");
    updateLayerUpdateTopic();
  }

  updateGeometry_();
  updateNormals_();

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
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Layer Update Topic",
      QString("Invalid topic: ") + e.what());
  } catch (const std::exception & e) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Layer Update Topic",
      QString("Failed to subscribe: ") + e.what());
  }
}

void NavMapDisplay::unsubscribeToLayerTopic()
{
  layer_subscription_.reset();
}

void NavMapDisplay::updateLayerUpdateTopic()
{
  // Re-hook only the layer subscription (main NavMap subscription is managed by the base class)
  unsubscribeToLayerTopic();
  subscribeToLayerTopic();
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

  // Exactly one data array must be non-empty and match the number of triangles
  const size_t n_tris = last_msg_->navcels_v0.size();
  const size_t n_u8 = msg->data_u8.size();
  const size_t n_f32 = msg->data_f32.size();
  const size_t n_f64 = msg->data_f64.size();
  const int non_empty = (n_u8 ? 1 : 0) + (n_f32 ? 1 : 0) + (n_f64 ? 1 : 0);

  if (non_empty != 1) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Layer Update",
      "Exactly one of data_u8 / data_f32 / data_f64 must be non-empty.");
    return;
  }
  const size_t eff_len = n_u8 ? n_u8 : (n_f32 ? n_f32 : n_f64);
  if (eff_len != n_tris) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Layer Update",
      QString("Layer size (%1) does not match number of triangles (%2)")
      .arg(eff_len).arg(n_tris));
    return;
  }

  applyOrCacheLayer_(*msg);

  if (currentSelectedLayer_() == msg->name) {
    updateGeometry_();
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
  }
  if (!prev.empty()) {
    layer_property_->setString(prev.c_str());
  }
}

void NavMapDisplay::applyOrCacheLayer_(const NavMapLayerMsg & layer)
{
  if (!last_msg_) {
    return;
  }
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

// Property slots
void NavMapDisplay::onLayerSelectionChanged()
{
  updateGeometry_();
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
  updateGeometry_();
  context_->queueRender();
}

void NavMapDisplay::onNormalScaleChanged()
{
  if (draw_normals_property_->getBool()) {
    updateNormals_();
    context_->queueRender();
  }
}

// Rendering pipeline

void NavMapDisplay::updateGeometry_()
{
  if (!context_ || !last_msg_) {return;}

  // Ensure scene objects exist
  if (!root_node_) {
    root_node_ = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  }
  if (!triangles_obj_) {
    triangles_obj_ = context_->getSceneManager()->createManualObject();
    triangles_obj_->setDynamic(true);
    root_node_->attachObject(triangles_obj_);
  }
  if (!normals_obj_) {
    normals_obj_ = context_->getSceneManager()->createManualObject();
    normals_obj_->setDynamic(true);
    root_node_->attachObject(normals_obj_);
  }

  triangles_obj_->clear();

  // Aliases
  const auto & X = last_msg_->positions_x;
  const auto & Y = last_msg_->positions_y;
  const auto & Z = last_msg_->positions_z;
  const auto & V0 = last_msg_->navcels_v0;
  const auto & V1 = last_msg_->navcels_v1;
  const auto & V2 = last_msg_->navcels_v2;

  if (X.empty() || Y.empty() || Z.empty() || V0.empty() || V1.empty() || V2.empty()) {
    return;
  }

  // Decide color mode: selected layer vs per-vertex RGBA
  const navmap_ros_interfaces::msg::NavMapLayer * selected_layer = nullptr;
  bool vertex_color_mode = false;

  const std::string sel = currentSelectedLayer_();
  auto it = layers_by_name_.find(sel);
  if (it != layers_by_name_.end()) {
    selected_layer = it->second;
  } else if (sel == "Color (vertex RGBA)") {
    vertex_color_mode = last_msg_->has_vertex_rgba &&
      last_msg_->colors_r.size() == X.size() &&
      last_msg_->colors_g.size() == X.size() &&
      last_msg_->colors_b.size() == X.size();
  }

  // For float layers, precompute max for scaling
  float max_val = 0.0f;
  bool is_u8 = false;
  if (selected_layer) {
    if (selected_layer->type == navmap_ros_interfaces::msg::NavMapLayer::U8 &&
      selected_layer->data_u8.size() == V0.size())
    {
      is_u8 = true;
    } else if (selected_layer->type == navmap_ros_interfaces::msg::NavMapLayer::F32 &&
      selected_layer->data_f32.size() == V0.size())
    {
      for (float v : selected_layer->data_f32) {
        max_val = std::max(max_val, v);
      }
    } else if (selected_layer->type == navmap_ros_interfaces::msg::NavMapLayer::F64 &&
      selected_layer->data_f64.size() == V0.size())
    {
      for (double v : selected_layer->data_f64) {
        max_val = std::max(max_val, static_cast<float>(v));
      }
    } else {
      selected_layer = nullptr;
    }
  }

  triangles_obj_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

  const float alpha = alpha_property_->getFloat();

  if (vertex_color_mode) {
    const auto & R = last_msg_->colors_r;
    const auto & G = last_msg_->colors_g;
    const auto & B = last_msg_->colors_b;
    const auto & A = last_msg_->colors_a;

    for (size_t t = 0; t < V0.size(); ++t) {
      const uint32_t i0 = V0[t], i1 = V1[t], i2 = V2[t];

      const Ogre::ColourValue c0(
        R[i0] / 255.f, G[i0] / 255.f, B[i0] / 255.f,
        (A.empty() ? alpha : (A[i0] / 255.f) * alpha));
      const Ogre::ColourValue c1(
        R[i1] / 255.f, G[i1] / 255.f, B[i1] / 255.f,
        (A.empty() ? alpha : (A[i1] / 255.f) * alpha));
      const Ogre::ColourValue c2(
        R[i2] / 255.f, G[i2] / 255.f, B[i2] / 255.f,
        (A.empty() ? alpha : (A[i2] / 255.f) * alpha));

      triangles_obj_->position(X[i0], Y[i0], Z[i0]); triangles_obj_->colour(c0);
      triangles_obj_->position(X[i1], Y[i1], Z[i1]); triangles_obj_->colour(c1);
      triangles_obj_->position(X[i2], Y[i2], Z[i2]); triangles_obj_->colour(c2);
    }
  } else if (selected_layer) {
    // Layer-based coloring (U8 occupancy or float heatmap)
    if (is_u8) {
      for (size_t t = 0; t < V0.size(); ++t) {
        const uint32_t i0 = V0[t], i1 = V1[t], i2 = V2[t];
        const Ogre::ColourValue col = colorFromU8(selected_layer->data_u8[t], alpha);
        triangles_obj_->position(X[i0], Y[i0], Z[i0]); triangles_obj_->colour(col);
        triangles_obj_->position(X[i1], Y[i1], Z[i1]); triangles_obj_->colour(col);
        triangles_obj_->position(X[i2], Y[i2], Z[i2]); triangles_obj_->colour(col);
      }
    } else if (selected_layer->type == navmap_ros_interfaces::msg::NavMapLayer::F32) {
      for (size_t t = 0; t < V0.size(); ++t) {
        const uint32_t i0 = V0[t], i1 = V1[t], i2 = V2[t];
        const Ogre::ColourValue col = colorFromHeat(selected_layer->data_f32[t], max_val, alpha);
        triangles_obj_->position(X[i0], Y[i0], Z[i0]); triangles_obj_->colour(col);
        triangles_obj_->position(X[i1], Y[i1], Z[i1]); triangles_obj_->colour(col);
        triangles_obj_->position(X[i2], Y[i2], Z[i2]); triangles_obj_->colour(col);
      }
    } else {  // F64
      for (size_t t = 0; t < V0.size(); ++t) {
        const uint32_t i0 = V0[t], i1 = V1[t], i2 = V2[t];
        const float v = static_cast<float>(selected_layer->data_f64[t]);
        const Ogre::ColourValue col = colorFromHeat(v, max_val, alpha);
        triangles_obj_->position(X[i0], Y[i0], Z[i0]); triangles_obj_->colour(col);
        triangles_obj_->position(X[i1], Y[i1], Z[i1]); triangles_obj_->colour(col);
        triangles_obj_->position(X[i2], Y[i2], Z[i2]); triangles_obj_->colour(col);
      }
    }
  } else {
    // Neutral gray fallback
    const Ogre::ColourValue col(0.7f, 0.7f, 0.7f, alpha);
    for (size_t t = 0; t < V0.size(); ++t) {
      const uint32_t i0 = V0[t], i1 = V1[t], i2 = V2[t];
      triangles_obj_->position(X[i0], Y[i0], Z[i0]); triangles_obj_->colour(col);
      triangles_obj_->position(X[i1], Y[i1], Z[i1]); triangles_obj_->colour(col);
      triangles_obj_->position(X[i2], Y[i2], Z[i2]); triangles_obj_->colour(col);
    }
  }

  triangles_obj_->end();
}

void NavMapDisplay::updateNormals_()
{
  if (!context_ || !last_msg_ || !normals_obj_) {return;}

  normals_obj_->clear();

  if (!draw_normals_property_->getBool()) {
    return;
  }

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


}  // namespace navmap_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(navmap_rviz_plugin::NavMapDisplay, rviz_common::Display)
