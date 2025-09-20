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

class NavMapToGridNode : public rclcpp::Node {
public:
  NavMapToGridNode()
  : Node("navmap_to_occgrid")
  {
    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("navmap_grid", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
      std::bind(&NavMapToGridNode::tick, this));
  }

private:
  void tick()
  {
    static bool init = false;
    static navmap::NavMap nm;
    if(!init) {
      auto v0 = nm.add_vertex({0, 0, 0});
      auto v1 = nm.add_vertex({1, 0, 0});
      auto v2 = nm.add_vertex({1, 1, 0});
      auto v3 = nm.add_vertex({0, 1, 0});
      auto c0 = nm.add_navcel(v0, v1, v2);
      auto c1 = nm.add_navcel(v0, v2, v3);
      auto s = nm.create_surface("map");
      nm.add_navcel_to_surface(s, c0);
      nm.add_navcel_to_surface(s, c1);
      nm.add_layer<uint8_t>("occupancy", "occ", "%", 0);
      nm.layer_set<uint8_t>("occupancy", c0, 100);
      nm.rebuild_geometry_accels();
      init = true;
    }
    auto grid = navmap_ros::to_occupancy_grid(nm);
    pub_->publish(grid);
  }
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavMapToGridNode>());
  rclcpp::shutdown();
  return 0;
}
