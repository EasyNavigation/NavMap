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

#include "navmap_core/NavMap.hpp"
#include "navmap_ros/conversions.hpp"
#include "navmap_ros/navmap_io.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

class SLAMServerNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SLAMServerNode)

  SLAMServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("slam_server_node", options)
  {
    navmap_pub_ = create_publisher<navmap_ros_interfaces::msg::NavMap>("navmap",
      rclcpp::QoS(1).transient_local().reliable());

    incoming_occ_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "incoming_occ_map", rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::UniquePtr msg) {
        RCLCPP_INFO(get_logger(), "Creating and publishing NavMap from OccupancyGrid");

        navmap_ = navmap_ros::from_occupancy_grid(*msg);

        navmap_msg_ = navmap_ros::to_msg(navmap_);
        navmap_msg_.header.frame_id = "map";
        navmap_msg_.header.stamp = now();
        navmap_pub_->publish(navmap_msg_);
      });

    incoming_pc2_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "incoming_pc2_map", rclcpp::QoS(100),
      [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
        RCLCPP_INFO(get_logger(), "Creating and publishing NavMap from PointCloud2");
        navmap_ros::BuildParams params;
        params.max_edge_len = 5.0f;
        params.min_angle_deg = 25.0f;
        params.max_slope_deg = 30.0f;
        params.resolution = 1.0f;
        navmap_ = navmap_ros::from_pointcloud2(*msg, navmap_msg_, params);

        navmap_msg_.header.frame_id = "map";
        navmap_msg_.header.stamp = now();
        navmap_pub_->publish(navmap_msg_);
      });

    savemap_srv_ = create_service<std_srvs::srv::Trigger>("savemap",
        [this](
          const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
          (void)request;
          (void)response;
          RCLCPP_INFO(get_logger(), "Saving NavMap from /tmp/map.navmap");

          navmap_ros::io::save_to_file(navmap_, "/tmp/map.navmap");
    });
  }

private:
  navmap::NavMap navmap_;
  navmap_ros_interfaces::msg::NavMap navmap_msg_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr incoming_occ_map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr incoming_pc2_map_sub_;
  rclcpp::Publisher<navmap_ros_interfaces::msg::NavMap>::SharedPtr navmap_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr savemap_srv_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto slam_server_node = SLAMServerNode::make_shared();
  rclcpp::spin(slam_server_node);

  return 0;
}
