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


#include "navmap_rviz_plugin/navmap_pose_tool.hpp"

#include <memory>
#include <string>
#include <utility>

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#include <OgreCamera.h>

#include <Eigen/Core>

#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/render_window.hpp"

#include "rviz_common/logging.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/view_controller.hpp"

namespace navmap_rviz_plugin
{

NavMapPoseTool::NavMapPoseTool()
: rviz_common::Tool(), arrow_(nullptr), angle_(0)
{
  projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
}

NavMapPoseTool::~NavMapPoseTool() = default;

void NavMapPoseTool::onInitialize()
{
  arrow_ = std::make_shared<rviz_rendering::Arrow>(
    scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
}

void NavMapPoseTool::activate()
{
  setStatus("Click and drag mouse to set position/orientation.");
  state_ = Position;
}

void NavMapPoseTool::deactivate()
{
  arrow_->getSceneNode()->setVisible(false);
}

static std::pair<bool, Ogre::Vector3>
rayHitOnNavMap(
  rviz_common::ViewportMouseEvent & event,
  rviz_common::DisplayContext * context)
{
  auto * view_controller = context->getViewManager()->getCurrent();
  if (!view_controller) {
    return {false, Ogre::Vector3::ZERO};
  }

  // 2) Cámara Ogre directamente
  Ogre::Camera * cam = view_controller->getCamera();
  if (!cam || !cam->getViewport()) {
    return {false, Ogre::Vector3::ZERO};
  }

  Ogre::Viewport * vp = cam->getViewport();

  const float nx = static_cast<float>(event.x) / static_cast<float>(vp->getActualWidth());
  const float ny = static_cast<float>(event.y) / static_cast<float>(vp->getActualHeight());
  const Ogre::Ray ray = cam->getCameraToViewportRay(nx, ny);

  Eigen::Vector3f o(ray.getOrigin().x, ray.getOrigin().y, ray.getOrigin().z);
  Eigen::Vector3f d(ray.getDirection().x, ray.getDirection().y, ray.getDirection().z);
  if (d.squaredNorm() == 0.0f) {
    return {false, Ogre::Vector3::ZERO};
  }
  d.normalize();

  ::navmap::NavCelId cid = 0;
  float t = 0.0f;
  Eigen::Vector3f hit;
  const bool ok = received_navmap.raycast(o, d, cid, t, hit);
  if (!ok) {
    return {false, Ogre::Vector3::ZERO};
  }

  return {true, Ogre::Vector3(hit.x(), hit.y(), hit.z())};
}

int NavMapPoseTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  std::pair<bool, Ogre::Vector3> hit = {false, Ogre::Vector3::ZERO};

  if (!received_navmap.surfaces.empty()) {
    hit = rayHitOnNavMap(event, context_);
  }

  if (!hit.first) {  // Fallback: If there is not an intersection with navmap, intersect with z = 0
    hit = projection_finder_->getViewportPointProjectionOnXYPlane(
      event.panel->getRenderWindow(), event.x, event.y);
  }

  if (event.leftDown()) {
    return processMouseLeftButtonPressed(hit);   // espera pair<bool, Ogre::Vector3>
  } else if (event.type == QEvent::MouseMove && event.left()) {
    return processMouseMoved(hit);
  } else if (event.leftUp()) {
    return processMouseLeftButtonReleased();
  }
  return 0;
}

int NavMapPoseTool::processMouseLeftButtonPressed(
  std::pair<bool,
  Ogre::Vector3> xy_plane_intersection)
{
  int flags = 0;
  assert(state_ == Position);
  if (xy_plane_intersection.first) {
    arrow_position_ = xy_plane_intersection.second;
    arrow_->setPosition(arrow_position_);

    state_ = Orientation;
    flags |= Render;
  }
  return flags;
}

int NavMapPoseTool::processMouseMoved(std::pair<bool, Ogre::Vector3> xy_plane_intersection)
{
  int flags = 0;
  if (state_ == Orientation) {
    // compute angle in x-y plane
    if (xy_plane_intersection.first) {
      angle_ = calculateAngle(xy_plane_intersection.second, arrow_position_);
      makeArrowVisibleAndSetOrientation(angle_);

      flags |= Render;
    }
  }

  return flags;
}

void NavMapPoseTool::makeArrowVisibleAndSetOrientation(double angle)
{
  arrow_->getSceneNode()->setVisible(true);

  // we need base_orient, since the arrow goes along the -z axis by default
  // (for historical reasons)
  Ogre::Quaternion orient_x = Ogre::Quaternion(
    Ogre::Radian(-Ogre::Math::HALF_PI),
    Ogre::Vector3::UNIT_Y);

  arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z) * orient_x);
}

int NavMapPoseTool::processMouseLeftButtonReleased()
{
  int flags = 0;
  if (state_ == Orientation) {
    onPoseSet(arrow_position_.x, arrow_position_.y, arrow_position_.z, angle_);
    flags |= (Finished | Render);
  }

  return flags;
}

double NavMapPoseTool::calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point)
{
  return atan2(start_point.y - end_point.y, start_point.x - end_point.x);
}

geometry_msgs::msg::Quaternion NavMapPoseTool::orientationAroundZAxis(double angle)
{
  auto orientation = geometry_msgs::msg::Quaternion();
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = sin(angle) / (2 * cos(angle / 2));
  orientation.w = cos(angle / 2);
  return orientation;
}

void NavMapPoseTool::logPose(
  std::string designation, geometry_msgs::msg::Point position,
  geometry_msgs::msg::Quaternion orientation, double angle, std::string frame)
{
  RVIZ_COMMON_LOG_INFO_STREAM(
    "Setting " << designation << " pose: Frame:" << frame << ", Position(" << position.x << ", " <<
      position.y << ", " << position.z << "), Orientation(" << orientation.x << ", " <<
      orientation.y << ", " << orientation.z << ", " << orientation.w <<
      ") = Angle: " << angle);
}

}  // namespace navmap_rviz_plugin
