// Copyright 2022 <@ivrolan>
#include <memory>
#include "follow-wall/SensingNode.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SensingNode>("sensing_node");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
