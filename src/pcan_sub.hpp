#pragma once
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PcanSubscriber : public rclcpp::Node
{
public:
    PcanSubscriber();

private:
    void TopicCallback(const std_msgs::msg::String & msg) const;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mSubscription;
};