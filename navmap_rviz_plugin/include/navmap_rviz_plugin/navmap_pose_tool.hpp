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


#ifndef NAVMAP_RVIZ_PLUGIN__NAVMAP_POSE_TOOL_HPP_
#define NAVMAP_RVIZ_PLUGIN__NAVMAP_POSE_TOOL_HPP_

#include <memory>
#include <string>
#include <utility>

#include <OgreVector.h>

#include <QCursor>  // NOLINT cpplint cannot handle include order here

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "rviz_common/tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include "navmap_rviz_plugin/NavMapDisplay.hpp"


namespace rviz_rendering
{
class Arrow;
}  // namespace rviz_rendering

namespace navmap_rviz_plugin
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC NavMapPoseTool : public rviz_common::Tool
{
public:
  NavMapPoseTool();

  ~NavMapPoseTool() override;

  void onInitialize() override;

  void activate() override;

  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

protected:
  virtual void onPoseSet(double x, double y, double z, double theta) = 0;

  geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle);

  void logPose(
    std::string designation,
    geometry_msgs::msg::Point position,
    geometry_msgs::msg::Quaternion orientation,
    double angle,
    std::string frame);

  std::shared_ptr<rviz_rendering::Arrow> arrow_;

  enum State
  {
    Position,
    Orientation
  };
  State state_;
  double angle_;

  Ogre::Vector3 arrow_position_;
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

private:
  int processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
  int processMouseMoved(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
  int processMouseLeftButtonReleased();
  void makeArrowVisibleAndSetOrientation(double angle);
  double calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point);
};

}  // namespace navmap_rviz_plugin

#endif  // NAVMAP_RVIZ_PLUGIN__NAVMAP_POSE_TOOL_HPP_
