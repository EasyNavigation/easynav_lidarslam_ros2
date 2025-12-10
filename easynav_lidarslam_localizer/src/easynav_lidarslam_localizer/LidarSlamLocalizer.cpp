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

  robot_frame_ = "base_link";
  std::string input_cloud, imu;
  node->declare_parameter(plugin_name + ".input_cloud", input_cloud);
  node->declare_parameter(plugin_name + ".imu", imu);
  node->declare_parameter(plugin_name + ".robot_frame", robot_frame_);
  node->get_parameter(plugin_name + ".input_cloud", input_cloud);
  node->get_parameter(plugin_name + ".imu", imu);
  node->get_parameter(plugin_name + ".robot_frame", robot_frame_);

  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  // Initialize the odometry message
  odom_.header.stamp = get_node()->now();
  odom_.header.frame_id = tf_info.map_frame;
  odom_.child_frame_id = tf_info.robot_frame;

  rclcpp::NodeOptions options_gb;
  options_gb.use_intra_process_comms(true);
  gb_slam_ = std::make_shared<graphslam::GraphBasedSlamComponent>(options_gb);

  rclcpp::NodeOptions options_sm;
  options_sm.use_intra_process_comms(true);
  std::vector<std::string> remappings = {
    "/input_cloud:=" + input_cloud,
    "/imu:=" + imu
  };
  options_sm.arguments(remappings);
  sm_comp_ = std::make_shared<graphslam::ScanMatcherComponent>(options_sm);

  // Override internal scanmatcher frame configuration from TFInfo
  sm_comp_->set_parameter(rclcpp::Parameter("global_frame_id", tf_info.map_frame));
  sm_comp_->set_parameter(rclcpp::Parameter("robot_frame_id", tf_info.robot_frame));
  sm_comp_->set_parameter(rclcpp::Parameter("odom_frame_id", tf_info.odom_frame));

  return {};
}


void LidarSlamLocalizer::update_rt(NavState & nav_state)
{
}

void LidarSlamLocalizer::update(NavState & nav_state)
{
  rclcpp::spin_some(sm_comp_);
  rclcpp::spin_some(gb_slam_);

  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

    tf_msg = RTTFBuffer::getInstance()->lookupTransform(
      tf_info.map_frame, tf_info.robot_frame, tf2::TimePointZero, tf2::durationFromSec(0.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_node()->get_logger(), "LidarSlamLocalizer::update: TF failed: %s", ex.what());
    return;
  }

  odom_.header.stamp = tf_msg.header.stamp;
  odom_.pose.pose.position.x = tf_msg.transform.translation.x;
  odom_.pose.pose.position.y = tf_msg.transform.translation.y;
  odom_.pose.pose.position.z = tf_msg.transform.translation.z;
  odom_.pose.pose.orientation = tf_msg.transform.rotation;

  nav_state.set("robot_pose", odom_);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::LidarSlamLocalizer, easynav::LocalizerMethodBase)
