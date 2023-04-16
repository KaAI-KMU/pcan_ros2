#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PcanPublisher : public rclcpp::Node
{
public:
    PcanPublisher();

private:
    void TimerCallback();

    rclcpp::TimerBase::SharedPtr mTimer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mPublisher;
    size_t mCount;
};