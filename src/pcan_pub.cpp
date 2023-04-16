#include "pcan_pub.hpp"

using namespace std::chrono_literals;

PcanPublisher::PcanPublisher()
: Node("PCAN_Publisher"), mCount(0)
{
  mPublisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
  mTimer = this->create_wall_timer(
    500ms, std::bind(&PcanPublisher::TimerCallback, this));
}

void PcanPublisher::TimerCallback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(mCount++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  mPublisher->publish(message);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcanPublisher>());
  rclcpp::shutdown();
  return 0;
}
