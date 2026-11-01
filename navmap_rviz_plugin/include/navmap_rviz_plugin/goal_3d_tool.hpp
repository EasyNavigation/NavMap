// Copyright 2026 Intelligent Robotics Lab
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

#ifndef NAVMAP_RVIZ_PLUGIN__GOAL_3D_TOOL_HPP_
#define NAVMAP_RVIZ_PLUGIN__GOAL_3D_TOOL_HPP_

#include <memory>

#include <OgreVector.h>

#include <QObject>  // NOLINT cpplint cannot handle include order here

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_rendering
{
class Arrow;
class Line;
}  // namespace rviz_rendering

namespace rviz_common
{
namespace properties
{
class FloatProperty;
class StringProperty;
class QosProfileProperty;
}  // namespace properties
}  // namespace rviz_common

namespace navmap_rviz_plugin
{

/// @brief A goal anywhere in space (aerial robots): on the line from the camera through the
/// cursor, at a distance the mouse wheel changes. Click to place it, drag to set its yaw.
class RVIZ_DEFAULT_PLUGINS_PUBLIC Goal3DTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  Goal3DTool();
  ~Goal3DTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private Q_SLOTS:
  void updateTopic();

private:
  /// The point at the current distance on the ray through (x, y); false without a camera.
  bool point_on_ray(int x, int y, Ogre::Vector3 & point) const;
  /// Where the ray through (x, y) meets the horizontal plane at height z.
  bool point_on_plane(int x, int y, double z, Ogre::Vector3 & point) const;
  void show(const Ogre::Vector3 & position, double yaw);
  void hide();
  void update_status();

  enum class State {Position, Orientation};
  State state_ {State::Position};
  Ogre::Vector3 position_;
  double yaw_ {0.0};

  std::shared_ptr<rviz_rendering::Arrow> arrow_;
  std::shared_ptr<rviz_rendering::Line> drop_line_;  ///< Down to z = 0: where it is in depth.
  std::shared_ptr<rviz_rendering::Line> foot_x_, foot_y_;  ///< A cross where the line lands.

  rviz_common::properties::FloatProperty * distance_property_;
  rviz_common::properties::FloatProperty * wheel_step_property_;
  rviz_common::properties::StringProperty * topic_property_;
  rviz_common::properties::QosProfileProperty * qos_profile_property_;
  rclcpp::QoS qos_profile_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace navmap_rviz_plugin

#endif  // NAVMAP_RVIZ_PLUGIN__GOAL_3D_TOOL_HPP_
