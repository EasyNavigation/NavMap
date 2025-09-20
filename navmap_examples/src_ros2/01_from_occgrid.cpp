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
