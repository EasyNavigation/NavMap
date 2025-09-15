#pragma once

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

namespace navmap_rviz_plugin {

class NavMapDisplay : public rviz_common::MessageFilterDisplay<navmap_ros_interfaces::msg::NavMap>
{
public:
  NavMapDisplay();
  ~NavMapDisplay() override;

protected:
  void onInitialize() override;
  void reset() override;
  void processMessage(const navmap_ros_interfaces::msg::NavMap::ConstSharedPtr msg) override;

private:
  // Helpers
  void clearScene();
  void rebuildGeometry();
  void updateLayerChoices();
  Ogre::ColourValue colorFromU8(uint8_t v) const;
  Ogre::ColourValue colorFromHeat(float value, float max_value) const;
  void onPropertyChanged();

  // Properties
  rviz_common::properties::EnumProperty* layer_property_;
  rviz_common::properties::BoolProperty* draw_normals_property_;
  rviz_common::properties::FloatProperty* normal_scale_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::StringProperty* info_property_;

  // Scene
  Ogre::ManualObject* triangles_obj_;
  Ogre::ManualObject* normals_obj_;
  Ogre::SceneNode*    root_node_;

  // Data
  navmap_ros_interfaces::msg::NavMap::ConstSharedPtr last_msg_;
  std::unordered_map<std::string, const navmap_ros_interfaces::msg::NavMapLayer*> layers_by_name_;
};

} // namespace navmap_rviz_plugin
