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

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "follow_wall_interfaces/msg/laser_info.hpp"
#include "follow_wall/ActuationNode.hpp"

using std::placeholders::_1;

ActuationNode::ActuationNode(const std::string & name)
: Node(name)
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/nav_vel", 10);
  laser_info_sub_ = create_subscription<follow_wall_interfaces::msg::LaserInfo>(
    "/follow_wall/data", 10, std::bind(&ActuationNode::laser_callback, this, _1));
}

void ActuationNode::laser_callback(const follow_wall_interfaces::msg::LaserInfo::SharedPtr msg) {}

void ActuationNode::do_work() {}
