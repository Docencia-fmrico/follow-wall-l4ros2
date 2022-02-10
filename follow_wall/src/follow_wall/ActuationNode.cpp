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
  //vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/key_op", 10);
  laser_info_sub_ = create_subscription<follow_wall_interfaces::msg::LaserInfo>(
    "/follow_wall/data", 10, std::bind(&ActuationNode::laser_callback, this, _1));
}

void ActuationNode::laser_callback(const follow_wall_interfaces::msg::LaserInfo::SharedPtr msg) {
  //TODO: change the private data.
  
  /*
  if (data says there is a wall in front) {
    state_ = FOLLOW
  }
  */
}

geometry_msgs::msg::Twist stop() {
  geometry_msgs::msg::Twist vel_msg;
  vel_msg.linear.x = 0.0;
  vel_msg.linear.y = 0.0;
  vel_msg.linear.z = 0.0;
  vel_msg.angular.x = 0.0;
  vel_msg.angular.y = 0.0;
  vel_msg.angular.z = 0.0;

  return vel_msg;
}

geometry_msgs::msg::Twist search() {
  geometry_msgs::msg::Twist vel_msg;
  vel_msg.linear.x = FOLLOWING_LINEAR_VEL;
  vel_msg.linear.y = 0.0;
  vel_msg.linear.z = 0.0;
  vel_msg.angular.x = 0.0;
  vel_msg.angular.y = 0.0;
  vel_msg.angular.z = 0.0;

  return vel_msg;
}

void ActuationNode::do_work() {
  geometry_msgs::msg::Twist vel_msg;

  switch(state_) {
    case SEARCH: //go forward until a wall is found.
      vel_msg = search();
      break;
    case FOLLOW: //follow the found wall.
      vel_msg = stop();
      //vel_msg = follow();
      break;
  }

  vel_pub_->publish(vel_msg);


  
}
