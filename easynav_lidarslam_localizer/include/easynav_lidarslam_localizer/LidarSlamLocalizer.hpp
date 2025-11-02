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
/// \brief Declaration of the LidarSlamLocalizer class, a default plugin implementation for localization.

#ifndef EASYNAV_LOCALIZER__LIDARSLAMLOCALIZER_HPP_
#define EASYNAV_LOCALIZER__LIDARSLAMLOCALIZER_HPP_

#include <expected>
#include "graph_based_slam/graph_based_slam_component.h"
#include "scanmatcher/scanmatcher_component.h"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "easynav_core/LocalizerMethodBase.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "sensor_msgs/msg/imu.hpp"


namespace easynav
{

/**
 * @class LidarSlamLocalizer
 * @brief A default "do-nothing" implementation of the LocalizerMethodBase.
 *
 * This class implements the interface required by the Easy Navigation framework
 * for localization but does not perform any actual computation. It is useful as
 * a placeholder, example, or fallback when no real localization plugin is configured.
 */
class LidarSlamLocalizer : public easynav::LocalizerMethodBase
{
public:
  /**
   * @brief Default constructor.
   */
  LidarSlamLocalizer() = default;

  /**
   * @brief Default destructor.
   */
  ~LidarSlamLocalizer() = default;

  /**
   * @brief Initialize the localization method.
   *
   * This override may be used to set up internal resources. By default, it simply succeeds.
   *
   * @return std::expected<void, std::string> Returns success or an error message.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Updates the localization estimate based on the current navigation state.
   *
   * This method is intended to run the localization logic and update the odometry.
   * In this implementation, the method updates the odometry based on the latest LidarSlam data.
   * LidarSlam data is used to set the robot's position in the UTM coordinate system.
   * The origin of the UTM coordinates is set when the first LidarSlam message is received.
   *
   * @param nav_state The current navigation state of the system.
   */
  virtual void update_rt(NavState & nav_state) override;

  /**
   * @brief Updates the localization estimate based on the current navigation state.
   *
   * This method is intended to run the localization logic and update the odometry.
   * In this implementation, the method updates the odometry based on the latest LidarSlam data.
   * LidarSlam data is used to set the robot's position in the UTM coordinate system.
   * The origin of the UTM coordinates is set when the first LidarSlam message is received.
   *
   * @param nav_state The current navigation state of the system.
   */
  virtual void update(NavState & nav_state) override;

private:
  /**
   * @brief Internal representation of the robot's current odometry.
   *
   * Stores the estimated position and velocity of the robot.
   */
  nav_msgs::msg::Odometry odom_;

  std::string robot_frame_;

  std::shared_ptr<graphslam::GraphBasedSlamComponent> gb_slam_;
  std::shared_ptr<graphslam::ScanMatcherComponent> sm_comp_;
};

}  // namespace easynav

#endif  // EASYNAV_LOCALIZER__LIDARSLAMLOCALIZER_HPP_
