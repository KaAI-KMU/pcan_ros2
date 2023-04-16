#include "pcan_sub.hpp"

using std::placeholders::_1;

PcanSubscriber::PcanSubscriber()
  : Node("PCAN_Subscriber")
{
  mSubscription = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&PcanSubscriber::TopicCallback, this, _1));
}

void PcanSubscriber::TopicCallback(const std_msgs::msg::String & msg) const
{
  RCLCPP_INFO(this->get_logger(), "Received CAN data: %s", msg.data.c_str());
}