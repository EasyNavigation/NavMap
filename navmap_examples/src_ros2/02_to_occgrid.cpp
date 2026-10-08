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
