#include "follow-wall-l4ros2/SensingNode.hpp"

using std::placeholders::_1;

SensingNode::SensingNode(const std::string& name) : Node(name)
{
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_raw", rclcpp::QoS(1).reliable(), std::bind(&SensingNode::callback, this, _1));

    pub_ = create_publisher<example_interfaces::msg::Int8MultiArray>("follow_wall_data", 10);
}


int get_val_from_d(int d){

    if (d < SAFE_DISTANCE - INTERVAL){
        return LESS;
    }

    if(d > SAFE_DISTANCE + INTERVAL){
        return GREATER;
    }

    return IN_RANGE;

}

void SensingNode::callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
    // the equation is angle_min + angle_increment * index = angle_to_measure
    RCLCPP_INFO(this->get_logger(), "Got msg\n");
    int index_front = (0 - msg->angle_min) / msg->angle_increment;
    int index_right = (-M_PI_2 - msg->angle_min) / msg->angle_increment;
    int index_back_45 = (-M_PI_2 - M_PI_4 - msg->angle_min) / msg->angle_increment;

    // TODO: Adding markers to visualize the measures with a blue, green and red line.

    float front_d = msg->ranges[index_front];
    float right_d = msg->ranges[index_right];
    float back_45_d = msg->ranges[index_back_45];

    example_interfaces::msg::Int8MultiArray msg_to_send;

    msg_to_send.data.push_back(get_val_from_d(front_d));
    msg_to_send.data.push_back(get_val_from_d(right_d));
    msg_to_send.data.push_back(get_val_from_d(back_45_d));

    pub_->publish(msg_to_send);

    RCLCPP_INFO(this->get_logger(), "Sent info to topic\n");

}
