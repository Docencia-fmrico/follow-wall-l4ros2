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

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "follow_wall/SensingNode.hpp"

#include "gtest/gtest.h"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    sub_ =
      create_subscription<follow_wall_interfaces::msg::LaserInfo>(
      "/follow_wall/data", rclcpp::QoS(10).reliable(),
      std::bind(&MinimalSubscriber::callback, this, _1));
  }

  void callback(const follow_wall_interfaces::msg::LaserInfo::SharedPtr msg)
  {
    msg_ = msg;
  }
  follow_wall_interfaces::msg::LaserInfo::SharedPtr msg_;
  rclcpp::Subscription<follow_wall_interfaces::msg::LaserInfo>::SharedPtr sub_;
};

TEST(sensing_test, greater_range_test)
{
  auto laser_node = rclcpp::Node::make_shared("laser_node_pub");
  auto publisher = laser_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan_raw", 10);
  auto node_sub = std::make_shared<MinimalSubscriber>();

  auto sensing_node = std::make_shared<SensingNode>("sensing_node");


  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(laser_node);
  executor.add_node(node_sub);
  executor.add_node(sensing_node);

  sensor_msgs::msg::LaserScan data_1;
  data_1.angle_min = -1.9198600053787231;
  data_1.angle_increment = 0.005774015095084906;
  std::vector<float> vect1(665, 10.0);
  data_1.ranges = vect1;
  publisher->publish(data_1);

  {
    rclcpp::Rate rate(10);
    auto start = laser_node->now();
    while ((laser_node->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_EQ(node_sub->msg_->front, GREATER);
  ASSERT_EQ(node_sub->msg_->right, GREATER);
  ASSERT_EQ(node_sub->msg_->front_right, GREATER);
}

TEST(sensing_test, in_range_test)
{
  auto laser_node = rclcpp::Node::make_shared("laser_node2_pub");
  auto publisher = laser_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan_raw", 10);
  auto node_sub = std::make_shared<MinimalSubscriber>();

  auto sensing_node = std::make_shared<SensingNode>("sensing_node2");


  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(laser_node);
  executor.add_node(node_sub);
  executor.add_node(sensing_node);

  sensor_msgs::msg::LaserScan data_3;
  data_3.angle_min = -1.9198600053787231;
  data_3.angle_increment = 0.005774015095084906;
  std::vector<float> vect3(665, SAFE_DISTANCE);
  data_3.ranges = vect3;
  publisher->publish(data_3);

  {
    rclcpp::Rate rate(10);
    auto start = laser_node->now();
    while ((laser_node->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_EQ(node_sub->msg_->front, IN_RANGE);
  ASSERT_EQ(node_sub->msg_->right, IN_RANGE);
  ASSERT_EQ(node_sub->msg_->front_right, IN_RANGE);
}

TEST(sensing_test, less_range_test)
{
  auto laser_node = rclcpp::Node::make_shared("laser_node3_pub");
  auto publisher = laser_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan_raw", 10);
  auto node_sub = std::make_shared<MinimalSubscriber>();

  auto sensing_node = std::make_shared<SensingNode>("sensing_node3");


  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(laser_node);
  executor.add_node(node_sub);
  executor.add_node(sensing_node);

  sensor_msgs::msg::LaserScan data_2;
  data_2.angle_min = -1.9198600053787231;
  data_2.angle_increment = 0.005774015095084906;
  std::vector<float> vect2(665, 0.0);
  data_2.ranges = vect2;
  publisher->publish(data_2);

  {
    rclcpp::Rate rate(10);
    auto start = laser_node->now();
    while ((laser_node->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_EQ(node_sub->msg_->front, LESS);
  ASSERT_EQ(node_sub->msg_->right, LESS);
  ASSERT_EQ(node_sub->msg_->front_right, LESS);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
