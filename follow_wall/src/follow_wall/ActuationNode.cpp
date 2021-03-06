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
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using std::placeholders::_1;

ActuationNode::ActuationNode(const std::string & name)
: rclcpp_lifecycle::LifecycleNode(name), state_(SEARCH_WALL)
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/nav_vel", 10);
  sensing_info_sub_ =
    create_subscription<follow_wall_interfaces::msg::LaserInfo>(
    "/follow_wall/data", rclcpp::QoS(10).reliable(),
    std::bind(&ActuationNode::sensing_callback, this, _1));
  static_turn_ = false;

  // RCLCPP_INFO(this->get_logger(), "node created\n");
}

void ActuationNode::sensing_callback(const follow_wall_interfaces::msg::LaserInfo::SharedPtr msg)
{
  msg_ = msg;
  // RCLCPP_INFO(this->get_logger(), "sense receiving\n");
}

CallbackReturnT ActuationNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring.");
  // TODO(someone): add parameters.
  // speed_ = get_parameter("speed").get_value<double>();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ActuationNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");
  vel_pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ActuationNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating.");
  vel_pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ActuationNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleanning Up.");
  vel_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ActuationNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down.");
  vel_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT ActuationNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down.");
  return CallbackReturnT::SUCCESS;
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
      if (!static_turn_) {
        vel_msg.linear.x = TURNING_LINEAR_VEL;
      } else {
        RCLCPP_INFO(this->get_logger(), "static TURN_LEFT");
      }
      vel_msg.angular.z = TURNING_ANGULAR_VEL;
      break;

    case TURN_RIGHT:
      if (!static_turn_) {
        vel_msg.linear.x = TURNING_LINEAR_VEL;
      } else {
        RCLCPP_INFO(this->get_logger(), "static TURN_RIGHT");
      }
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

  static_turn_ = (msg_->front == CLOSE || msg_->front_right == CLOSE);
}
