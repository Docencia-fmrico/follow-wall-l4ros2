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
#ifndef FOLLOW_WALL__ACTUATIONNODE_HPP_
#define FOLLOW_WALL__ACTUATIONNODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "follow_wall_interfaces/msg/laser_info.hpp"

#define SAFE_DISTANCE 0.4
#define INTERVAL 0.05

class ActuationNode : public rclcpp::Node
{
public:
  explicit ActuationNode(const std::string & name);

  void do_work();

private:
  void laser_callback(const follow_wall_interfaces::msg::LaserInfo::SharedPtr msg);

private:
  rclcpp::Subscription<follow_wall_interfaces::msg::LaserInfo>::SharedPtr laser_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};


#endif  // FOLLOW_WALL__ACTUATIONNODE_HPP_
