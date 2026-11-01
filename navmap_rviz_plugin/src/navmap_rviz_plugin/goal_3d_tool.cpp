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

#include "navmap_rviz_plugin/goal_3d_tool.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "OgreCamera.h"
#include "OgrePlane.h"
#include "OgreRay.h"
#include "OgreSceneNode.h"
#include "OgreViewport.h"
#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/line.hpp"

namespace navmap_rviz_plugin
{

namespace
{

bool camera_ray(rviz_common::DisplayContext * context, int x, int y, Ogre::Ray & ray)
{
  auto * view = context->getViewManager()->getCurrent();
  Ogre::Camera * camera = view ? view->getCamera() : nullptr;
  if (!camera || !camera->getViewport()) {
    return false;
  }
  Ogre::Viewport * viewport = camera->getViewport();
  ray = camera->getCameraToViewportRay(
    static_cast<float>(x) / static_cast<float>(viewport->getActualWidth()),
    static_cast<float>(y) / static_cast<float>(viewport->getActualHeight()));
  return true;
}

}  // namespace

Goal3DTool::Goal3DTool()
: qos_profile_(5)
{
  shortcut_key_ = 'h';
  distance_property_ = new rviz_common::properties::FloatProperty(
    "Distance", 10.0, "Distance from the camera to the goal (m); the mouse wheel changes it.",
    getPropertyContainer());
  distance_property_->setMin(0.5);
  wheel_step_property_ = new rviz_common::properties::FloatProperty(
    "Wheel step", 1.0, "Distance change per wheel notch (m).", getPropertyContainer());
  wheel_step_property_->setMin(0.01);
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "goal_pose", "The topic on which to publish goals.", getPropertyContainer(),
    SLOT(updateTopic()), this);
  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);
}

Goal3DTool::~Goal3DTool() = default;

void Goal3DTool::onInitialize()
{
  arrow_ = std::make_shared<rviz_rendering::Arrow>(
    scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(0.0f, 0.8f, 1.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
  drop_line_ = std::make_shared<rviz_rendering::Line>(scene_manager_, nullptr);
  drop_line_->setColor(0.0f, 0.8f, 1.0f, 0.6f);
  drop_line_->setVisible(false);
  foot_x_ = std::make_shared<rviz_rendering::Line>(scene_manager_, nullptr);
  foot_y_ = std::make_shared<rviz_rendering::Line>(scene_manager_, nullptr);
  for (const auto & line : {foot_x_, foot_y_}) {
    line->setColor(0.0f, 0.8f, 1.0f, 1.0f);
    line->setVisible(false);
  }
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
  setName("3D Goal Pose");
  updateTopic();
}

void Goal3DTool::updateTopic()
{
  rclcpp::Node::SharedPtr node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), qos_profile_);
  clock_ = node->get_clock();
}

void Goal3DTool::activate()
{
  state_ = State::Position;
  update_status();
}

void Goal3DTool::deactivate()
{
  hide();
}

void Goal3DTool::hide()
{
  arrow_->getSceneNode()->setVisible(false);
  for (const auto & line : {drop_line_, foot_x_, foot_y_}) {
    line->setVisible(false);
  }
}

void Goal3DTool::update_status()
{
  setStatus(
    QString("Distance %1 m (mouse wheel). Click to place the goal, drag to set its yaw.")
    .arg(distance_property_->getFloat(), 0, 'f', 1));
}

bool Goal3DTool::point_on_ray(int x, int y, Ogre::Vector3 & point) const
{
  Ogre::Ray ray;
  if (!camera_ray(context_, x, y, ray)) {
    return false;
  }
  point = ray.getPoint(distance_property_->getFloat());
  return true;
}

bool Goal3DTool::point_on_plane(int x, int y, double z, Ogre::Vector3 & point) const
{
  Ogre::Ray ray;
  if (!camera_ray(context_, x, y, ray)) {
    return false;
  }
  const auto hit = ray.intersects(Ogre::Plane(Ogre::Vector3::UNIT_Z, static_cast<float>(z)));
  if (!hit.first) {
    return false;
  }
  point = ray.getPoint(hit.second);
  return true;
}

void Goal3DTool::show(const Ogre::Vector3 & position, double yaw)
{
  arrow_->setPosition(position);
  // The arrow points along -z by default
  arrow_->setOrientation(
    Ogre::Quaternion(Ogre::Radian(yaw), Ogre::Vector3::UNIT_Z) *
    Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y));
  arrow_->getSceneNode()->setVisible(true);
  const Ogre::Vector3 foot(position.x, position.y, 0.0f);
  drop_line_->setPoints(position, foot);
  const Ogre::Vector3 dx(0.5f, 0.0f, 0.0f), dy(0.0f, 0.5f, 0.0f);
  foot_x_->setPoints(foot - dx, foot + dx);
  foot_y_->setPoints(foot - dy, foot + dy);
  for (const auto & line : {drop_line_, foot_x_, foot_y_}) {
    line->setVisible(true);
  }
}

int Goal3DTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.type == QEvent::Wheel && state_ == State::Position) {
    const float notches = static_cast<float>(event.wheel_delta) / 120.0f;
    distance_property_->setValue(
      std::max(0.5f, distance_property_->getFloat() + notches * wheel_step_property_->getFloat()));
    update_status();
    Ogre::Vector3 p;
    if (point_on_ray(event.x, event.y, p)) {
      position_ = p;
      show(position_, yaw_);
    }
    return Render;
  }

  if (state_ == State::Position) {
    Ogre::Vector3 p;
    if (!point_on_ray(event.x, event.y, p)) {
      return 0;
    }
    position_ = p;
    show(position_, yaw_);
    if (event.leftDown()) {
      state_ = State::Orientation;
    }
    return Render;
  }

  // Orientation: towards the cursor, on the goal's horizontal plane
  Ogre::Vector3 p;
  if ((event.type == QEvent::MouseMove && event.left()) &&
    point_on_plane(event.x, event.y, position_.z, p) &&
    (p - position_).length() > 1e-3f)
  {
    yaw_ = std::atan2(p.y - position_.y, p.x - position_.x);
    show(position_, yaw_);
    return Render;
  }
  if (event.leftUp()) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = clock_->now();
    goal.header.frame_id = context_->getFixedFrame().toStdString();
    goal.pose.position.x = position_.x;
    goal.pose.position.y = position_.y;
    goal.pose.position.z = position_.z;
    goal.pose.orientation.z = std::sin(yaw_ / 2.0);
    goal.pose.orientation.w = std::cos(yaw_ / 2.0);
    RVIZ_COMMON_LOG_INFO_STREAM(
      "Setting 3D goal: Frame:" << goal.header.frame_id << ", Position(" <<
        goal.pose.position.x << ", " << goal.pose.position.y << ", " << goal.pose.position.z <<
        "), Yaw: " << yaw_);
    publisher_->publish(goal);
    state_ = State::Position;
    hide();
    return Finished | Render;
  }
  return 0;
}

}  // namespace navmap_rviz_plugin

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(navmap_rviz_plugin::Goal3DTool, rviz_common::Tool)
