#include "pcan_sub.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcanSubscriber>());
  rclcpp::shutdown();
  return 0;
}