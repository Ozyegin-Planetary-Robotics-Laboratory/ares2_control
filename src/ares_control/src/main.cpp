#include <rclcpp/rclcpp.hpp>
#include "ares_control/JoystickInterface.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // Log node initialization
  RCLCPP_INFO(rclcpp::get_logger("JoystickNode"), "Initializing Joystick Interface Node...");

  auto joystick_node = std::make_shared<ares_control::JoystickInterface>();

  // Log that the node is now spinning
  RCLCPP_INFO(rclcpp::get_logger("JoystickNode"), "Spinning the Joystick Interface Node...");
  
  rclcpp::spin(joystick_node);

  rclcpp::shutdown();
  
  RCLCPP_INFO(rclcpp::get_logger("JoystickNode"), "Joystick Interface Node has been shut down.");
  
  return 0;
}
