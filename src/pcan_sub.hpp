#pragma once
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

class PcanSubscriber : public rclcpp::Node
{
public:
    PcanSubscriber();

private:
    void TopicCallback(const std_msgs::msg::ByteMultiArray & msg);
    std::string GetIDString(const unsigned char* data_ptr) const;
    
    void Parser111(const unsigned char* data_ptr);
    void Parser710(const unsigned char* data_ptr);
    void Parser711(const unsigned char* data_ptr);
    double mBrake;
    double mSteering;
    double mThrottle;

    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr mSubscription;
};