#include "navmap_rviz_plugin/NavMapDisplay.hpp"

#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/validate_floats.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace navmap_rviz_plugin
{

NavMapDisplay::NavMapDisplay()
: layer_property_(nullptr)
  , draw_normals_property_(nullptr)
  , normal_scale_property_(nullptr)
  , alpha_property_(nullptr)
  , info_property_(nullptr)
  , triangles_obj_(nullptr)
  , normals_obj_(nullptr)
  , root_node_(nullptr)
{
  layer_property_ = new rviz_common::properties::EnumProperty(
    "Layer", "",
    "Select the layer to colorize triangles.",
    this);

  draw_normals_property_ = new rviz_common::properties::BoolProperty(
    "Draw Normals", false, "Draw a normal per triangle.", this);
  normal_scale_property_ = new rviz_common::properties::FloatProperty(
    "Normal Length", 0.2, "Visual length of normals.", this);
  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "Transparency for triangles (0..1).", this);
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  info_property_ = new rviz_common::properties::StringProperty(
    "Info", "", "Readonly info about the last NavMap.", this);
  info_property_->setReadOnly(true);

  // Connect property changes to rebuild without requiring Q_OBJECT/SLOT
  QObject::connect(layer_property_, &rviz_common::properties::Property::changed,
    [this]() {onPropertyChanged();});
  QObject::connect(draw_normals_property_, &rviz_common::properties::Property::changed,
    [this]() {onPropertyChanged();});
  QObject::connect(normal_scale_property_, &rviz_common::properties::Property::changed,
    [this]() {onPropertyChanged();});
  QObject::connect(alpha_property_, &rviz_common::properties::Property::changed,
    [this]() {onPropertyChanged();});
}

NavMapDisplay::~NavMapDisplay()
{
  clearScene();
}

void NavMapDisplay::onInitialize()
{
  MFDClass::onInitialize();
  root_node_ = scene_node_->createChildSceneNode();
  triangles_obj_ = context_->getSceneManager()->createManualObject();
  triangles_obj_->setDynamic(true);
  normals_obj_ = context_->getSceneManager()->createManualObject();
  normals_obj_->setDynamic(true);
  root_node_->attachObject(triangles_obj_);
  root_node_->attachObject(normals_obj_);
}

void NavMapDisplay::reset()
{
  MFDClass::reset();
  clearScene();
  last_msg_.reset();
}

void NavMapDisplay::clearScene()
{
  if (triangles_obj_) {triangles_obj_->clear();}
  if (normals_obj_) {normals_obj_->clear();}
}

void NavMapDisplay::onPropertyChanged()
{
  rebuildGeometry();
}

void NavMapDisplay::updateLayerChoices()
{
  layer_property_->clearOptions();
  layers_by_name_.clear();

  if (!last_msg_) {
    return;
  }

  int idx = 0;

  // Option for per-vertex RGBA if message provides valid arrays
  const bool has_vertex_rgba =
    last_msg_->has_vertex_rgba &&
    last_msg_->colors_r.size() == last_msg_->positions_x.size() &&
    last_msg_->colors_g.size() == last_msg_->positions_x.size() &&
    last_msg_->colors_b.size() == last_msg_->positions_x.size() &&
    last_msg_->colors_a.size() == last_msg_->positions_x.size();

  if (has_vertex_rgba) {
    const std::string display = "Color (vertex RGBA)";
    layer_property_->addOptionStd(display, idx++);
    layers_by_name_[display] = nullptr;  // nullptr marks the special mode
  }

  // Real layers from the message
  for (const auto & layer : last_msg_->layers) {
    layer_property_->addOptionStd(layer.name, idx);
    layers_by_name_[layer.name] = &layer;
    ++idx;
  }

  // Pick default if none selected yet
  if (idx > 0 && layer_property_->getStdString().empty()) {
    if (has_vertex_rgba) {
      layer_property_->setString("Color (vertex RGBA)");
    } else {
      layer_property_->setString(QString::fromStdString(last_msg_->layers.front().name));
    }
  }
}

Ogre::ColourValue NavMapDisplay::colorFromU8(uint8_t v) const
{
  if (v == 0) {return Ogre::ColourValue(0.5f, 0.5f, 0.5f, alpha_property_->getFloat());}
  if (v == 255) {return Ogre::ColourValue(0.0f, 0.39f, 0.0f, alpha_property_->getFloat());}
  if (v == 254) {return Ogre::ColourValue(0.0f, 0.0f, 0.0f, alpha_property_->getFloat());}
  float occ = static_cast<float>(v) / 253.0f;
  float c = 1.0f - occ;
  return Ogre::ColourValue(c, c, c, alpha_property_->getFloat());
}

Ogre::ColourValue NavMapDisplay::colorFromHeat(float value, float max_value) const
{
  float a = alpha_property_->getFloat();
  if (max_value <= 1e-9f) {return Ogre::ColourValue(0.0f, 1.0f, 0.0f, a);}
  float t = std::max(0.0f, std::min(1.0f, value / max_value));
  float r = t;
  float g = 1.0f - t;
  float b = 0.0f;
  return Ogre::ColourValue(r, g, b, a);
}

void NavMapDisplay::processMessage(const navmap_ros_interfaces::msg::NavMap::ConstSharedPtr msg)
{
  last_msg_ = msg;

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position,
      orientation))
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "Frame",
              "Failed to transform from frame " + QString::fromStdString(msg->header.frame_id));
  } else {
    root_node_->setPosition(position);
    root_node_->setOrientation(orientation);
  }

  updateLayerChoices();

  std::ostringstream oss;
  size_t nv = msg->positions_x.size();
  size_t nt = msg->navcels_v0.size();
  oss << "Vertices: " << nv << "  Triangles: " << nt << "  Surfaces: " << msg->surfaces.size() <<
    "  Layers: " << msg->layers.size();
  info_property_->setStdString(oss.str());

  rebuildGeometry();
}

void NavMapDisplay::rebuildGeometry()
{
  clearScene();
  if (!last_msg_) {return;}

  const auto & X = last_msg_->positions_x;
  const auto & Y = last_msg_->positions_y;
  const auto & Z = last_msg_->positions_z;
  const auto & V0 = last_msg_->navcels_v0;
  const auto & V1 = last_msg_->navcels_v1;
  const auto & V2 = last_msg_->navcels_v2;

  if (X.empty() || V0.empty()) {return;}

  // Determine layer coloring
  const navmap_ros_interfaces::msg::NavMapLayer * selected_layer = nullptr;
  bool vertex_color_mode = false;
  auto sel = layer_property_->getStdString();
  auto it = layers_by_name_.find(sel);
  if (it != layers_by_name_.end()) {
    if (it->second == nullptr && sel == std::string("Color (vertex RGBA)")) {
      vertex_color_mode = true;
    } else {
      selected_layer = it->second;
    }
  }

  // Precompute max for float layers
  float max_val = 0.0f;
  bool is_u8 = false;
  if (selected_layer) {
    if (selected_layer->type == 0 && selected_layer->data_u8.size() == V0.size()) {
      is_u8 = true;
    } else if ((selected_layer->type == 1 && selected_layer->data_f32.size() == V0.size())) {
      for (auto v : selected_layer->data_f32) {
        max_val = std::max(max_val, v);
      }
    } else if ((selected_layer->type == 2 && selected_layer->data_f64.size() == V0.size())) {
      for (auto v : selected_layer->data_f64) {
        max_val = std::max(max_val, static_cast<float>(v));
      }
    }
  }

  // Build triangles
  triangles_obj_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
  for (size_t i = 0; i < V0.size(); ++i) {
    uint32_t i0 = V0[i], i1 = V1[i], i2 = V2[i];
    if (i0 >= X.size() || i1 >= X.size() || i2 >= X.size()) {continue;}

    if (vertex_color_mode &&
      last_msg_->has_vertex_rgba &&
      last_msg_->colors_r.size() == X.size() &&
      last_msg_->colors_g.size() == X.size() &&
      last_msg_->colors_b.size() == X.size() &&
      last_msg_->colors_a.size() == X.size())
    {

      const float a_scale = alpha_property_->getFloat();
      auto make_col = [&](uint32_t idx)->Ogre::ColourValue {
          float r = static_cast<float>(last_msg_->colors_r[idx]) / 255.0f;
          float g = static_cast<float>(last_msg_->colors_g[idx]) / 255.0f;
          float b = static_cast<float>(last_msg_->colors_b[idx]) / 255.0f;
          float a = (static_cast<float>(last_msg_->colors_a[idx]) / 255.0f) * a_scale;
          return Ogre::ColourValue(r, g, b, a);
        };
      Ogre::ColourValue c0 = make_col(i0);
      Ogre::ColourValue c1 = make_col(i1);
      Ogre::ColourValue c2 = make_col(i2);

      triangles_obj_->position(X[i0], Y[i0], Z[i0]); triangles_obj_->colour(c0);
      triangles_obj_->position(X[i1], Y[i1], Z[i1]); triangles_obj_->colour(c1);
      triangles_obj_->position(X[i2], Y[i2], Z[i2]); triangles_obj_->colour(c2);
      continue;
    }

    Ogre::ColourValue col(0.7f, 0.7f, 0.7f, alpha_property_->getFloat());
    if (selected_layer) {
      if (is_u8 && i < selected_layer->data_u8.size()) {
        col = colorFromU8(selected_layer->data_u8[i]);
      } else if (selected_layer->type == 1 && i < selected_layer->data_f32.size()) {
        col = colorFromHeat(selected_layer->data_f32[i], max_val);
      } else if (selected_layer->type == 2 && i < selected_layer->data_f64.size()) {
        col = colorFromHeat(static_cast<float>(selected_layer->data_f64[i]), max_val);
      }
    }

    triangles_obj_->position(X[i0], Y[i0], Z[i0]); triangles_obj_->colour(col);
    triangles_obj_->position(X[i1], Y[i1], Z[i1]); triangles_obj_->colour(col);
    triangles_obj_->position(X[i2], Y[i2], Z[i2]); triangles_obj_->colour(col);
  }
  triangles_obj_->end();

  // Build normals if requested
  if (draw_normals_property_->getBool()) {
    float len = normal_scale_property_->getFloat();
    normals_obj_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    for (size_t i = 0; i < V0.size(); ++i) {
      uint32_t i0 = V0[i], i1 = V1[i], i2 = V2[i];
      if (i0 >= X.size() || i1 >= X.size() || i2 >= X.size()) {continue;}
      Ogre::Vector3 p0(X[i0], Y[i0], Z[i0]);
      Ogre::Vector3 p1(X[i1], Y[i1], Z[i1]);
      Ogre::Vector3 p2(X[i2], Y[i2], Z[i2]);
      Ogre::Vector3 c = (p0 + p1 + p2) / 3.0f;
      Ogre::Vector3 n = (p1 - p0).crossProduct(p2 - p0);
      if (n.squaredLength() > 1e-12) {n.normalise();}
      Ogre::Vector3 tip = c + len * n;
      normals_obj_->position(c); normals_obj_->colour(Ogre::ColourValue(0, 0, 1, 1));
      normals_obj_->position(tip); normals_obj_->colour(Ogre::ColourValue(0, 0, 1, 1));
    }
    normals_obj_->end();
  }
}

} // namespace navmap_rviz_plugin

PLUGINLIB_EXPORT_CLASS(navmap_rviz_plugin::NavMapDisplay, rviz_common::Display)
