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

// Loads a .navmap file from disk and republishes it periodically so
// late-joining subscribers -- e.g. RViz2's NavMapDisplay, added via
// navmap_rviz_plugin -- reliably receive it.
//
// Originally this published once on a transient-local topic (matching
// navmap_ros/src/slam_server_app.cpp's "navmap" publisher) and relied on
// DDS's durability-service replay for late joiners. That reproducibly never
// delivered for this specific message type: confirmed empirically with a
// throwaway rclpy publisher (no C++ of this file involved) that
// NavMap+TRANSIENT_LOCAL never reaches a subscriber even locally, while
// NavMap+VOLATILE with periodic publishing, and TRANSIENT_LOCAL with a
// plain std_msgs/String, both work fine in the same environment -- pointing
// at a rmw/FastDDS quirk specific to this message type's transient-local
// history replay, not a bug in navmap_tools. Periodic republish sidesteps it
// entirely: any subscriber alive for at least one period receives the map,
// with plain, well-exercised QoS. See easynav_gis_tool.md for the full
// diagnostic trail.

#include <chrono>
#include <iostream>
#include <string>

#include "navmap_ros/navmap_io.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

struct Args
{
  std::string input;
  std::string topic = "navmap";
  std::string frame_id;
  double rate_hz = 1.0;
};

bool parse_args(int argc, char * argv[], Args & args)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char * flag) -> const char * {
        if (i + 1 >= argc) {
          std::cerr << "missing value for " << flag << "\n";
          return nullptr;
        }
        return argv[++i];
      };
    if (arg == "--input") {
      const char * v = next("--input"); if (!v) {return false;} args.input = v;
    } else if (arg == "--topic") {
      const char * v = next("--topic"); if (!v) {return false;} args.topic = v;
    } else if (arg == "--frame-id") {
      const char * v = next("--frame-id"); if (!v) {return false;} args.frame_id = v;
    } else if (arg == "--rate") {
      const char * v = next("--rate"); if (!v) {return false;}
      args.rate_hz = std::stod(v);
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return false;
    }
  }
  if (args.input.empty()) {
    std::cerr << "usage: navmap_publisher --input <map.navmap> [--topic navmap] "
      "[--frame-id map] [--rate 1.0]\n";
    return false;
  }
  if (args.rate_hz <= 0.0) {
    std::cerr << "--rate must be positive\n";
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char * argv[])
{
  Args args;
  if (!parse_args(argc, argv, args)) {
    return 1;
  }

  navmap_ros_interfaces::msg::NavMap msg;
  std::error_code ec;
  if (!navmap_ros::io::load_msg_from_file(args.input, msg, &ec)) {
    std::cerr << "failed to load " << args.input << ": " << ec.message() << "\n";
    return 1;
  }

  // argv has already been fully consumed by parse_args() above and is not
  // ROS syntax (no --ros-args), so it is deliberately not passed here --
  // doing so makes rclcpp try (and noisily fail) to lex it as ROS arguments.
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("navmap_publisher");

  if (!args.frame_id.empty()) {
    msg.header.frame_id = args.frame_id;
  }

  auto pub = node->create_publisher<navmap_ros_interfaces::msg::NavMap>(
    args.topic, rclcpp::QoS(1).reliable());

  RCLCPP_INFO(
    node->get_logger(),
    "Publishing %s (%zu vertices, %zu triangles) on '%s' at %.2f Hz, frame '%s'",
    args.input.c_str(), msg.positions_x.size(), msg.navcels_v0.size(), args.topic.c_str(),
    args.rate_hz, msg.header.frame_id.c_str());

  const auto period = std::chrono::duration<double>(1.0 / args.rate_hz);
  auto timer = node->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [&node, &pub, &msg]() {
      msg.header.stamp = node->now();
      pub->publish(msg);
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
