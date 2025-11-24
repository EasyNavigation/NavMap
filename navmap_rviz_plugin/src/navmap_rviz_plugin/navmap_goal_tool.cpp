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


#include "navmap_rviz_plugin/navmap_goal_tool.hpp"

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"

namespace navmap_rviz_plugin
{

NavMapGoalTool::NavMapGoalTool()
: navmap_rviz_plugin::NavMapPoseTool(), qos_profile_(5)
{
  shortcut_key_ = 'g';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "goal_pose",
    "The topic on which to publish goals.",
    getPropertyContainer(), SLOT(updateTopic()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);
}

NavMapGoalTool::~NavMapGoalTool() = default;

void NavMapGoalTool::onInitialize()
{
  NavMapPoseTool::onInitialize();
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
  setName("NavMap Goal Pose");
  updateTopic();
}

void NavMapGoalTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = raw_node->
    template create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), qos_profile_);
  clock_ = raw_node->get_clock();
}

void NavMapGoalTool::onPoseSet(double x, double y, double z, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = clock_->now();
  goal.header.frame_id = fixed_frame;

  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;

  goal.pose.orientation = orientationAroundZAxis(theta);

  logPose("goal", goal.pose.position, goal.pose.orientation, theta, fixed_frame);

  publisher_->publish(goal);
}

}  // namespace navmap_rviz_plugin

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(navmap_rviz_plugin::NavMapGoalTool, rviz_common::Tool)
