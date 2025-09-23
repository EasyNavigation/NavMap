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


#include "navmap_rviz_plugin/NavMapDisplay.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OgreEntity.h>
#include <OgreMesh.h>
#include <OgreMeshManager.h>
#include <OgreHardwareVertexBuffer.h>
#include <OgreHardwareBufferManager.h>
#include <OgreMaterialManager.h>
#include <OgreSubMesh.h>
#include <OgreMaterial.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreVertexIndexData.h>
#include <OgreRoot.h>

#include <rclcpp/exceptions.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/uniform_string_stream.hpp>
#include <rviz_common/validate_floats.hpp>

#include <QCoreApplication>
#include <cmath>
#include <sstream>

namespace
{

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
  if (v == 0) {return Ogre::ColourValue(0.5f, 0.5f, 0.5f, alpha);}
  if (v == 255) {return Ogre::ColourValue(0.0f, 0.39f, 0.0f, alpha);}
  if (v == 254) {return Ogre::ColourValue(0.0f, 0.0f, 0.0f, alpha);}
  float occ = static_cast<float>(v) / 253.0f;
  float c = 1.0f - occ;
  return Ogre::ColourValue(c, c, c, alpha);
}

inline Ogre::ColourValue colorFromHeat(float value, float max_value, float alpha)
{
  if (max_value <= 1e-9f) {return Ogre::ColourValue(0.0f, 1.0f, 0.0f, alpha);}
  float t = std::max(0.0f, std::min(1.0f, value / max_value));
  return Ogre::ColourValue(t, 1.0f - t, 0.0f, alpha);
}

} // namespace

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

  color_scheme_property_ = new rviz_common::properties::EnumProperty(
    "Color Scheme", "Heat",
    "Color mapping for the active layer. U8: Occupancy. Float: Heat or Rainbow.",
    this, SLOT(onColorSchemeChanged()));

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
  last_navmap_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_layer_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  setStatus(rviz_common::properties::StatusProperty::Ok, "NavMap Topic", "waiting for messages…");
  setStatus(rviz_common::properties::StatusProperty::Ok, "Layer Update", "waiting for updates…");

  destroyMesh_();
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

  last_msg_ = std::make_shared<NavMapMsg>(*msg);

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

  // Build/refresh mesh and color buffer
  ensureMeshBuilt_();
  updateColorsOnly_();
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

void NavMapDisplay::incomingLayer(const NavMapLayerMsg::ConstSharedPtr & msg)
{
  if (!last_msg_) {
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Layer Update",
      "Received a layer before any NavMap; it will be applied upon the next NavMap message.");
    return;
  }

  const size_t n_tris = last_msg_->navcels_v0.size();
  const size_t n_u8 = msg->data_u8.size();
  const size_t n_f32 = msg->data_f32.size();
  const size_t n_f64 = msg->data_f64.size();
  const int non_empty = (n_u8 ? 1 : 0) + (n_f32 ? 1 : 0) + (n_f64 ? 1 : 0);

  if (non_empty != 1) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Layer Update",
      "Exactly one of data_u8 / data_f32 / data_f64 must be non-empty.");
    return;
  }
  const size_t eff_len = n_u8 ? n_u8 : (n_f32 ? n_f32 : n_f64);
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
    (msg->type == navmap_ros_interfaces::msg::NavMapLayer::F64) ? "F64" : "UNKNOWN";

  const size_t len = !msg->data_u8.empty() ? msg->data_u8.size() :
    !msg->data_f32.empty() ? msg->data_f32.size() :
    msg->data_f64.size();

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

// ----- Mesh/Entity: build once, recolor fast -----

void NavMapDisplay::ensureMeshBuilt_()
{
  if (mesh_built_ || !context_ || !last_msg_) {return;}

  const auto & X = last_msg_->positions_x;
  const auto & Y = last_msg_->positions_y;
  const auto & Z = last_msg_->positions_z;
  const auto & V0 = last_msg_->navcels_v0;
  const auto & V1 = last_msg_->navcels_v1;
  const auto & V2 = last_msg_->navcels_v2;

  if (X.empty() || Y.empty() || Z.empty() || V0.empty() || V1.empty() || V2.empty()) {
    return;
  }

  // 1) Build a ManualObject once to get vertex/index streams, with a neutral colour
  Ogre::ManualObject * mo = context_->getSceneManager()->createManualObject();
  mo->setDynamic(false);
  mo->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

  const Ogre::ColourValue neutral(0.7f, 0.7f, 0.7f, alpha_property_->getFloat());
  for (size_t t = 0; t < V0.size(); ++t) {
    const uint32_t i0 = V0[t], i1 = V1[t], i2 = V2[t];
    mo->position(X[i0], Y[i0], Z[i0]); mo->colour(neutral);
    mo->position(X[i1], Y[i1], Z[i1]); mo->colour(neutral);
    mo->position(X[i2], Y[i2], Z[i2]); mo->colour(neutral);
  }
  mo->end();

  // 2) Convert to Mesh and create Entity
  static int mesh_counter = 0;
  const std::string mesh_name = "NavMapMesh_" + std::to_string(++mesh_counter);
  Ogre::MeshPtr m = mo->convertToMesh(mesh_name);

  // Material with vertex-colour tracking
  Ogre::MaterialPtr base = Ogre::MaterialManager::getSingleton().getByName("BaseWhiteNoLighting");
  Ogre::MaterialPtr mat = base->clone(mesh_name + "/mat");
  mat->setLightingEnabled(false);
  // For alpha < 1 you may uncomment:
  // auto * pass = mat->getTechnique(0)->getPass(0);
  // pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  // pass->setDepthWriteEnabled(false);
  mat->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT | Ogre::TVC_DIFFUSE);

  // Destroy temp ManualObject
  mo->detachFromParent();
  context_->getSceneManager()->destroyManualObject(mo);

  // Create entity and attach to our node
  entity_ = context_->getSceneManager()->createEntity(mesh_name);
  entity_->setMaterialName(mat->getName());
  if (!root_node_) {
    root_node_ = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  }
  root_node_->attachObject(entity_);

  // Keep mesh/shared pointers
  mesh_ = m;

  // 3) Move VES_DIFFUSE to its own vertex buffer (dedicated colour stream)
  Ogre::SubMesh * sub = mesh_->getSubMesh(0);
  Ogre::VertexData * vdata = sub->useSharedVertices ? mesh_->sharedVertexData : sub->vertexData;
  Ogre::VertexDeclaration * decl = vdata->vertexDeclaration;
  Ogre::VertexBufferBinding * bind = vdata->vertexBufferBinding;

  const size_t vertex_count = vdata->vertexCount;

  // Remove any existing colour element (usually in source 0). Some Ogre builds provide
  // removeElement by semantic; if not, rebuild via iteration—here we try semantic removal first.
  if (decl->findElementBySemantic(Ogre::VES_DIFFUSE)) {
#if OGRE_VERSION_MAJOR > 1 || (OGRE_VERSION_MAJOR == 1 && (OGRE_VERSION_MINOR > 12 || \
    (OGRE_VERSION_MINOR == 12 && OGRE_VERSION_PATCH >= 0)))
    // Many vendors carry removeElement(VES_DIFFUSE)
    decl->removeElement(Ogre::VES_DIFFUSE);
#else
    // Fallback: rebuild declaration without VES_DIFFUSE
    Ogre::VertexDeclaration * newDecl = context_->getSceneManager()->createVertexDeclaration();
    for (unsigned short s = 0; s < decl->getElementCount(); ++s) {
      const Ogre::VertexElement & e = decl->getElement(s);
      if (e.getSemantic() == Ogre::VES_DIFFUSE) {continue;}
      newDecl->addElement(e.getSource(), e.getOffset(), e.getType(), e.getSemantic(), e.getIndex());
    }
    // Copy back
    *decl = *newDecl;
#endif
  }

  // Choose a free binding source for colours (avoid clashing with positions/normals)
  unsigned short COLOR_SRC = 1;
  while (bind->isBufferBound(COLOR_SRC)) {++COLOR_SRC;}

  // Create the dynamic colour buffer (one 32-bit colour per vertex)
  const Ogre::VertexElementType col_type = Ogre::VET_COLOUR_ARGB; // we'll pack ARGB
  decl->addElement(COLOR_SRC, /*offset=*/0, col_type, Ogre::VES_DIFFUSE);

  Ogre::HardwareVertexBufferSharedPtr colour_vbuf =
    Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
      Ogre::VertexElement::getTypeSize(col_type), // should be 4
      vertex_count,
      Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

  bind->setBinding(COLOR_SRC, colour_vbuf);

  // Store for fast recolour
  colour_vbuf_ = colour_vbuf;
  colour_vbuf_source_ = COLOR_SRC;

  mesh_built_ = true;
}

void NavMapDisplay::destroyMesh_()
{
  if (!context_) {return;}

  if (entity_) {
    entity_->detachFromParent();
    context_->getSceneManager()->destroyEntity(entity_);
    entity_ = nullptr;
  }
  if (mesh_) {
    Ogre::MeshManager::getSingleton().remove(mesh_->getHandle());
    mesh_.reset();
  }
  colour_vbuf_.reset();
  mesh_built_ = false;
}

void NavMapDisplay::updateColorsOnly_()
{
  if (!mesh_built_ || !last_msg_ || !entity_ || !mesh_) {return;}

  const auto & V0 = last_msg_->navcels_v0;
  const auto & V1 = last_msg_->navcels_v1;
  const auto & V2 = last_msg_->navcels_v2;
  if (V0.empty() || V1.empty() || V2.empty()) {return;}

  // Resolve active layer and scheme
  const std::string sel = currentSelectedLayer_();
  const float alpha = alpha_property_->getFloat();
  const bool use_rainbow = (color_scheme_property_->getStdString() == "Rainbow");

  const NavMapLayerMsg * selected_layer = nullptr;
  bool vertex_color_mode = false;

  auto it = layers_by_name_.find(sel);
  if (it != layers_by_name_.end()) {
    selected_layer = it->second;
  } else if (sel == "Color (vertex RGBA)") {
    vertex_color_mode = last_msg_->has_vertex_rgba &&
      last_msg_->colors_r.size() == last_msg_->positions_x.size() &&
      last_msg_->colors_g.size() == last_msg_->positions_x.size() &&
      last_msg_->colors_b.size() == last_msg_->positions_x.size();
  }

  float max_val = 0.0f;
  bool is_u8 = false;
  if (selected_layer) {
    if (selected_layer->type == NavMapLayerMsg::U8 &&
      selected_layer->data_u8.size() == V0.size())
    {
      is_u8 = true;
    } else if (selected_layer->type == NavMapLayerMsg::F32 &&
      selected_layer->data_f32.size() == V0.size())
    {
      for (float v : selected_layer->data_f32) {
        max_val = std::max(max_val, v);
      }
    } else if (selected_layer->type == NavMapLayerMsg::F64 &&
      selected_layer->data_f64.size() == V0.size())
    {
      for (double v : selected_layer->data_f64) {
        max_val = std::max(max_val, static_cast<float>(v));
      }
    } else {
      selected_layer = nullptr; // fallback to neutral grey
    }
  }

  // Access our dedicated colour buffer
  if (!colour_vbuf_) {return;}

  const size_t expected_vertices = V0.size() * 3;

  // If geometry changed, rebuild mesh & buffers
  {
    Ogre::SubMesh * sub = mesh_->getSubMesh(0);
    Ogre::VertexData * vdata = sub->useSharedVertices ? mesh_->sharedVertexData : sub->vertexData;
    if (vdata->vertexCount != expected_vertices) {
      destroyMesh_();
      ensureMeshBuilt_();
      if (!mesh_built_ || !colour_vbuf_) {return;}
    }
  }

  // Lock and write one packed colour per vertex (buffer is colour-only → stride is 4)
  unsigned char * base = static_cast<unsigned char *>(
    colour_vbuf_->lock(Ogre::HardwareBuffer::HBL_DISCARD));
  uint32_t * p = reinterpret_cast<uint32_t *>(base);

  auto packARGB = [&](const Ogre::ColourValue & c) -> uint32_t {
    // Pack into ARGB to match VET_COLOUR_ARGB used at creation
      return Ogre::VertexElement::convertColourValue(c, Ogre::VET_COLOUR_ARGB);
    };

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

      *p++ = packARGB(c0);
      *p++ = packARGB(c1);
      *p++ = packARGB(c2);
    }
  } else if (selected_layer) {
    if (is_u8) {
      for (size_t t = 0; t < V0.size(); ++t) {
        const Ogre::ColourValue col = colorFromU8(selected_layer->data_u8[t], alpha);
        const uint32_t packed = packARGB(col);
        *p++ = packed; *p++ = packed; *p++ = packed;
      }
    } else if (selected_layer->type == NavMapLayerMsg::F32) {
      for (size_t t = 0; t < V0.size(); ++t) {
        const Ogre::ColourValue col = use_rainbow ?
          colorFromRainbow(selected_layer->data_f32[t], max_val, alpha) :
          colorFromHeat   (selected_layer->data_f32[t], max_val, alpha);
        const uint32_t packed = packARGB(col);
        *p++ = packed; *p++ = packed; *p++ = packed;
      }
    } else { // F64
      for (size_t t = 0; t < V0.size(); ++t) {
        const float v = static_cast<float>(selected_layer->data_f64[t]);
        const Ogre::ColourValue col = use_rainbow ?
          colorFromRainbow(v, max_val, alpha) :
          colorFromHeat   (v, max_val, alpha);
        const uint32_t packed = packARGB(col);
        *p++ = packed; *p++ = packed; *p++ = packed;
      }
    }
  } else {
    // Fallback to neutral grey
    const uint32_t packed = packARGB(Ogre::ColourValue(0.7f, 0.7f, 0.7f, alpha));
    for (size_t t = 0; t < V0.size(); ++t) {
      *p++ = packed; *p++ = packed; *p++ = packed;
    }
  }

  colour_vbuf_->unlock();

  // No extra invalidation is necessary; OGRE will render new colours next frame.
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

void NavMapDisplay::updateColorSchemeOptions_()
{
  const QString prev = color_scheme_property_->getString();
  color_scheme_property_->clearOptions();

  const std::string sel = currentSelectedLayer_();
  auto it = layers_by_name_.find(sel);

  if (it != layers_by_name_.end()) {
    const auto * L = it->second;
    if (L->type == navmap_ros_interfaces::msg::NavMapLayer::U8) {
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
