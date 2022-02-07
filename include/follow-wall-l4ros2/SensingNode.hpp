#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "example_interfaces/msg/int8_multi_array.hpp"

enum{
    LESS = -1,
    IN_RANGE,
    GREATER
};

#define SAFE_DISTANCE 0.4
#define INTERVAL 0.05

class SensingNode : public rclcpp::Node
{
    public:
        SensingNode(const std::string& name);

    private:
        void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const;

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
        rclcpp::Publisher<example_interfaces::msg::Int8MultiArray>::SharedPtr pub_;
};
