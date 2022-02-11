// Copyright 2019 Intelligent Robotics Lab
//
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
#ifndef FOLLOW_WALL__SENSINGNODE_HPP_
#define FOLLOW_WALL__SENSINGNODE_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "example_interfaces/msg/int8_multi_array.hpp"
#include "follow_wall_interfaces/msg/laser_info.hpp"

enum
{
  LESS = -1,
  IN_RANGE,
  GREATER
};

#define SAFE_DISTANCE 0.4
#define INTERVAL 0.05

class SensingNode : public rclcpp::Node
{
public:
  explicit SensingNode(const std::string & name);

private:
  void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const;

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<follow_wall_interfaces::msg::LaserInfo>::SharedPtr pub_;
};
#endif  // FOLLOW_WALL__SENSINGNODE_HPP_
