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
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Implementation of the LidarSlamLocalizer class.

#include <expected>
#include "easynav_lidarslam_localizer/LidarSlamLocalizer.hpp"

#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{

std::expected<void, std::string> LidarSlamLocalizer::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  // Initialize the odometry message
  odom_.header.stamp = get_node()->now();
  odom_.header.frame_id = "map";
  odom_.child_frame_id = "base_link";

  rclcpp::NodeOptions options_gb;
  options_gb.use_intra_process_comms(true);
  gb_slam_ = std::make_shared<graphslam::GraphBasedSlamComponent>(options_gb);

  std::string input_cloud, imu;
  node->declare_parameter(plugin_name + ".input_cloud", input_cloud);
  node->declare_parameter(plugin_name + ".imu", imu);
  node->get_parameter(plugin_name + ".input_cloud", input_cloud);
  node->get_parameter(plugin_name + ".imu", imu);

  rclcpp::NodeOptions options_sm;
  options_sm.use_intra_process_comms(true);
   std::vector<std::string> remappings = {
    "/input_cloud:=" + input_cloud,
    "/imu:=" + imu
  };
  options_sm.arguments(remappings);
  sm_comp_ = std::make_shared<graphslam::ScanMatcherComponent>(options_sm);

  return {};
}


void LidarSlamLocalizer::update_rt(NavState & nav_state)
{
}

void LidarSlamLocalizer::update(NavState & nav_state)
{
  rclcpp::spin_some(sm_comp_);
  rclcpp::spin_some(gb_slam_);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::LidarSlamLocalizer, easynav::LocalizerMethodBase)
