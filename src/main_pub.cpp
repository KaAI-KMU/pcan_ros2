#include "pcan_pub.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcanPublisher>());
  rclcpp::shutdown();
  return 0;
}
