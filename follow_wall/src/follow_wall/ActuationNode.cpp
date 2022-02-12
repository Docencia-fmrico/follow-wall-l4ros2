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
: Node(name), state_(SEARCH_WALL)
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/nav_vel", 10);
  sensing_info_sub_ = create_subscription<follow_wall_interfaces::msg::LaserInfo>(
    "/follow_wall/data", 10, std::bind(&ActuationNode::sensing_callback, this, _1));
}

void ActuationNode::sensing_callback(const follow_wall_interfaces::msg::LaserInfo::SharedPtr msg)
{
  msg_ = msg;
  // RCLCPP_INFO(this->get_logger(), "I heard something");
}

void ActuationNode::tick()
{
  if (msg_ == nullptr) {
    return;
  }

  geometry_msgs::msg::Twist vel_msg;
  vel_msg.linear.x = 0.0;
  vel_msg.linear.y = 0.0;
  vel_msg.linear.z = 0.0;
  vel_msg.angular.x = 0.0;
  vel_msg.angular.y = 0.0;
  vel_msg.angular.z = 0.0;

  this->update_state();
  switch (state_) {
    case SEARCH_WALL:
      RCLCPP_INFO(this->get_logger(), "state: %s", "SEARCH_WALL");
      break;
    case GO_STRAIGHT:
      RCLCPP_INFO(this->get_logger(), "state: %s", "GO_STRAIGHT");
      break;
    case TURN_LEFT:
      RCLCPP_INFO(this->get_logger(), "state: %s", "TURN_LEFT");
      break;
    case TURN_RIGHT:
      RCLCPP_INFO(this->get_logger(), "state: %s", "TURN_RIGHT");
      break;
  }

  switch (state_) {
    case SEARCH_WALL:
    case GO_STRAIGHT:
      vel_msg.linear.x = FOLLOWING_LINEAR_VEL;
      break;

    case TURN_LEFT:
      vel_msg.linear.x = TURNING_LINEAR_VEL;
      vel_msg.angular.z = TURNING_ANGULAR_VEL;
      break;

    case TURN_RIGHT:
      vel_msg.linear.x = TURNING_LINEAR_VEL;
      vel_msg.angular.z = -TURNING_ANGULAR_VEL;
      break;
  }

  vel_pub_->publish(vel_msg);
}

void ActuationNode::update_state()
{
  switch (state_) {
    case SEARCH_WALL:
      if (msg_->front == CLOSE || msg_->front_right == CLOSE || msg_->right == CLOSE) {
        state_ = TURN_LEFT;
        return;
      }
      return;
      break;

    case GO_STRAIGHT:
      switch (msg_->front) {
        case OKEY:
        case FAR:
          if (msg_->right == FAR && msg_->front_right == FAR) {
            state_ = TURN_RIGHT;
            return;
          }
          if (msg_->right == CLOSE || msg_->front_right == CLOSE) {
            state_ = TURN_LEFT;
            return;
          }
          return;
          break;

        case CLOSE:
          state_ = TURN_LEFT;
          return;
      }
      break;

    case TURN_LEFT:
      if (msg_->front == FAR) {
        if (msg_->right != CLOSE && msg_->front_right != CLOSE) {
          state_ = GO_STRAIGHT;
          return;
        }
      }
      return;
      break;

    case TURN_RIGHT:
      if (msg_->front == FAR) {
        if (msg_->right != FAR || msg_->front_right != FAR) {
          state_ = GO_STRAIGHT;
          return;
        }
      }
      return;
      break;
  }
}
