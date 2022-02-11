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
  last_data_ = *msg;

  /* //TODO when msg implemented.
  if (msg->front == IN_RANGE || msg->front == LESS) {
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

geometry_msgs::msg::Twist forward() {
  geometry_msgs::msg::Twist vel_msg;
  vel_msg.linear.x = FOLLOWING_LINEAR_VEL;
  vel_msg.linear.y = 0.0;
  vel_msg.linear.z = 0.0;
  vel_msg.angular.x = 0.0;
  vel_msg.angular.y = 0.0;
  vel_msg.angular.z = 0.0;

  return vel_msg;
}

geometry_msgs::msg::Twist turn_left() {
  geometry_msgs::msg::Twist vel_msg;
  vel_msg.linear.x = FOLLOWING_LINEAR_VEL;
  vel_msg.linear.y = 0.0;
  vel_msg.linear.z = 0.0;
  vel_msg.angular.x = 0.0;
  vel_msg.angular.y = 0.0;
  vel_msg.angular.z = 0.0;

  return vel_msg;
}



enum state state_transitioner(enum state state, follow_wall_interfaces::msg::LaserInfo data) {
  if (state == SEARCH && (data.front == LESS || data.front == IN_RANGE)) {
    state = TURN;
  }
  if (state == TURN && data.right == IN_RANGE) {
    state = FORWARD;
  }
  if (state == FORWARD) { //3 possibilities: closed corner, open corner or keep going forward.
    if ((data.front == LESS || data.front == IN_RANGE) &&
      (data.right == LESS || data.right == IN_RANGE)) {
      state = CLOSED_CORNER;
    } else if (data.right == GREATER) {
      state = OPEN_CORNER;
    }
  }
  if (state == CLOSED_CORNER && (data.right == IN_RANGE || data.right == LESS) &&
    (data.front == GREATER)) {
    state = FORWARD;
  }
  if (state == OPEN_CORNER && (data.right != GREATER)) {
    state = FORWARD;
  }

  return state;
}

void ActuationNode::do_work() {
  geometry_msgs::msg::Twist vel_msg;

  //transitions between states.
  state_ = state_transitioner(state_, last_data_);

  //state machine:
  switch(state_) {
    case SEARCH: //go forward until a wall is found.
      vel_msg = forward();
      break;
    case TURN: //turn until the wall is in the right.
      vel_msg = turn_left();
      break;
    case FORWARD: //follow the found wall.
      vel_msg = forward(); //TODO: correct the direction turning to the left or to the right.
      break;
    case CLOSED_CORNER: //turn left almost static.
      vel_msg = stop(); //TODO.
      break;
    case OPEN_CORNER: //turn right almost static.
      vel_msg = stop(); //TODO.
      break;
  }

  vel_pub_->publish(vel_msg);


  
}
