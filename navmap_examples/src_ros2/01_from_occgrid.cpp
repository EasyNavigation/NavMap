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


#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"

using std::placeholders::_1;

class GridToNavMapNode : public rclcpp::Node {
public:
  GridToNavMapNode()
  : Node("navmap_from_occgrid")
  {
    sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10, std::bind(&GridToNavMapNode::cb, this, _1));
  }

private:
  void cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    navmap::NavMap nm = navmap_ros::from_occupancy_grid(*msg);

    size_t sidx{}; navmap::NavCelId cid{}; Eigen::Vector3f bary, hit;
    if(nm.locate_navcel(Eigen::Vector3f(0.5f, 0.5f, 0.5f), sidx, cid, bary, &hit)) {
      RCLCPP_INFO(this->get_logger(), "hit on surface %zu cell %u", sidx, (unsigned)cid);
    }
  }
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridToNavMapNode>());
  rclcpp::shutdown();
  return 0;
}
