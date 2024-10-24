#include "io/omniperception/omniperception.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto perception = std::make_shared<omniperception_subscriber::Omniperception>();
  perception->start();
  // 主程序的其他逻辑...
  rclcpp::shutdown();
  return 0;
}