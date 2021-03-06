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

#include <algorithm>
#include <string>
#include "follow_wall/SensingNode.hpp"

using std::placeholders::_1;

SensingNode::SensingNode(const std::string & name)
: Node(name)
{
  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan_raw", rclcpp::QoS(1).reliable(),
    std::bind(&SensingNode::callback, this, _1));

  pub_ = create_publisher<follow_wall_interfaces::msg::LaserInfo>(
    "/follow_wall/data", 10);
}


int get_val_from_d(float d)
{
  if (d < SAFE_DISTANCE - INTERVAL) {
    return LESS;
  }

  if (d > SAFE_DISTANCE + INTERVAL) {
    return GREATER;
  }

  return IN_RANGE;
}

float get_min_dist(const sensor_msgs::msg::LaserScan::SharedPtr data, int data_index)
{
  return std::min(
    std::min(data->ranges[data_index], data->ranges[data_index + 1]),
    data->ranges[data_index - 1]);
}

void SensingNode::callback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
  // the equation is angle_min + angle_increment * index = angle_to_measure
  RCLCPP_INFO(this->get_logger(), "Got msg\n");
  int index_front = (0 - msg->angle_min) / msg->angle_increment;
  int index_right = (-M_PI_2 - msg->angle_min) / msg->angle_increment;
  int index_front_right =
    (-M_PI_4 - msg->angle_min) / msg->angle_increment;

  // with a blue, green and red line.

  float front_d = get_min_dist(msg, index_front);
  float right_d = get_min_dist(msg, index_right);
  float front_right_d = get_min_dist(msg, index_front_right);

  follow_wall_interfaces::msg::LaserInfo msg_to_send;

  msg_to_send.front = get_val_from_d(front_d);
  msg_to_send.right = get_val_from_d(right_d);
  msg_to_send.front_right = get_val_from_d(front_right_d);

  pub_->publish(msg_to_send);

  RCLCPP_INFO(this->get_logger(), "Sent info to topic\n");
}
