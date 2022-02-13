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
#include "follow_wall/SensingNode.hpp"
#include "follow_wall/ActuationNode.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto sensing_node = std::make_shared<SensingNode>("sensing_node");

  auto actuation_node = std::make_shared<ActuationNode>("actuation_node");

  rclcpp::Rate loop_rate(50ms);

  while (rclcpp::ok()) {
    actuation_node->tick();

    rclcpp::spin_some(sensing_node);
    rclcpp::spin_some(actuation_node->get_node_base_interface());

    loop_rate.sleep();
  }

  rclcpp::shutdown();
}
