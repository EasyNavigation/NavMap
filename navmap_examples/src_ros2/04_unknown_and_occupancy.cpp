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

// Unknown values and log-odds occupancy: a simulated 2D lidar circles a 20 x 10 m floor with a few
// boxes. Each ray clears the NavCels it crosses (miss) and marks the one it ends on (hit), so the
// unknown floor (grey) turns free (white) or occupied (black). An "elevation" layer has a patch
// never surveyed, NaN, declared as its unknown value (translucent grey in RViz).
//
// Publishes the NavMap once on "navmap" and the layers on "navmap_updates".

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "navmap_core/Iterators.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_core/Occupancy.hpp"
#include "navmap_ros/conversions.hpp"
#include "navmap_ros_interfaces/msg/nav_map.hpp"
#include "navmap_ros_interfaces/msg/nav_map_layer.hpp"
#include "rclcpp/rclcpp.hpp"

using navmap::NavCelId;

namespace
{

struct Box {float x0, y0, x1, y1;};

// Distance along (dx, dy) from (x, y) to the first box, or max_range.
float cast(const std::vector<Box> & boxes, float x, float y, float dx, float dy, float max_range)
{
  float best = max_range;
  for (const auto & b : boxes) {
    float t0 = 0.0f, t1 = max_range;
    for (const auto & [o, d, lo, hi] : {std::array<float, 4>{x, dx, b.x0, b.x1},
        std::array<float, 4>{y, dy, b.y0, b.y1}})
    {
      if (std::abs(d) < 1e-9f) {
        if (o < lo || o > hi) {t0 = max_range + 1;}
        continue;
      }
      float ta = (lo - o) / d, tb = (hi - o) / d;
      if (ta > tb) {std::swap(ta, tb);}
      t0 = std::max(t0, ta);
      t1 = std::min(t1, tb);
    }
    if (t0 <= t1 && t0 < best) {best = t0;}
  }
  return best;
}

}  // namespace

class UnknownOccupancyNode : public rclcpp::Node
{
public:
  UnknownOccupancyNode()
  : Node("navmap_unknown_occupancy")
  {
    // 20 x 10 m floor, 0.2 m cells (two triangles each)
    const int nx = 100, ny = 50;
    const float res = 0.2f;
    const std::size_t s = nm_.create_surface("map");
    for (int j = 0; j <= ny; ++j) {
      for (int i = 0; i <= nx; ++i) {
        nm_.add_vertex({i * res, j * res, 0.0f});
      }
    }
    auto v = [&](int i, int j) {return static_cast<uint32_t>(j * (nx + 1) + i);};
    for (int j = 0; j < ny; ++j) {
      for (int i = 0; i < nx; ++i) {
        nm_.add_navcel_to_surface(s, nm_.add_navcel(v(i, j), v(i + 1, j), v(i + 1, j + 1)));
        nm_.add_navcel_to_surface(s, nm_.add_navcel(v(i, j), v(i + 1, j + 1), v(i, j + 1)));
      }
    }
    nm_.rebuild_geometry_accels();

    // Elevation: a gentle wave, except a patch never surveyed (NaN = unknown)
    nm_.add_layer<float>("elevation", "terrain height", "m", 0.0f);
    nm_.set_layer_unknown("elevation", std::numeric_limits<double>::quiet_NaN());
    for (NavCelId c = 0; c < nm_.navcels.size(); ++c) {
      const auto p = nm_.navcel_centroid(c);
      const bool surveyed = !(p.x() > 14.0f && p.y() > 6.0f);
      nm_.layer_set<float>(
        "elevation", c, surveyed ? 0.5f + 0.5f * std::sin(p.x() * 0.6f) * std::cos(p.y() * 0.5f) :
        std::numeric_limits<float>::quiet_NaN());
    }

    occ_ = std::make_unique<navmap::OccupancyLayer>(nm_);  // all unknown

    boxes_ = {{3, 2, 4, 3}, {8, 6, 9.5f, 7}, {12, 1.5f, 12.5f, 4}, {16, 3, 17, 4}};

    rclcpp::QoS latched(1);
    latched.transient_local().reliable();
    map_pub_ = create_publisher<navmap_ros_interfaces::msg::NavMap>("navmap", latched);
    layer_pub_ = create_publisher<navmap_ros_interfaces::msg::NavMapLayer>("navmap_updates", 10);

    std_msgs::msg::Header h;
    h.frame_id = "map";
    h.stamp = now();
    map_pub_->publish(navmap_ros::to_msg(nm_, h));
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] {step();});
  }

private:
  void step()
  {
    // Robot on a circle around the map centre; 360 rays, 6 m range
    const float a = 0.05f * static_cast<float>(tick_++);
    const Eigen::Vector3f robot(10.0f + 6.0f * std::cos(a), 5.0f + 3.0f * std::sin(a), 0.0f);
    for (int k = 0; k < 360; ++k) {
      const float th = static_cast<float>(k) * static_cast<float>(M_PI) / 180.0f;
      const float dx = std::cos(th), dy = std::sin(th);
      const float range = 6.0f;
      const float d = cast(boxes_, robot.x(), robot.y(), dx, dy, range);
      const Eigen::Vector3f end(robot.x() + d * dx, robot.y() + d * dy, 0.0f);
      // Every NavCel the ray crosses is free, but the one it ends on
      NavCelId last = std::numeric_limits<NavCelId>::max();
      navmap::for_each_on_segment(
        nm_, robot, end, [&](NavCelId c, float, float t_out) {
          if (t_out < 1.0f) {occ_->miss(c);} else {last = c;}
        });
      if (last != std::numeric_limits<NavCelId>::max()) {
        if (d < range) {occ_->hit(last);} else {occ_->miss(last);}
      }
    }
    std_msgs::msg::Header h;
    h.frame_id = "map";
    h.stamp = now();
    layer_pub_->publish(navmap_ros::to_msg(nm_, occ_->name(), h));
    if (tick_ % 10 == 1) {
      layer_pub_->publish(navmap_ros::to_msg(nm_, "elevation", h));
      map_pub_->publish(navmap_ros::to_msg(nm_, h));  // late joiners see the current state
    }
  }

  navmap::NavMap nm_;
  std::unique_ptr<navmap::OccupancyLayer> occ_;
  std::vector<Box> boxes_;
  rclcpp::Publisher<navmap_ros_interfaces::msg::NavMap>::SharedPtr map_pub_;
  rclcpp::Publisher<navmap_ros_interfaces::msg::NavMapLayer>::SharedPtr layer_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int tick_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UnknownOccupancyNode>());
  rclcpp::shutdown();
  return 0;
}
