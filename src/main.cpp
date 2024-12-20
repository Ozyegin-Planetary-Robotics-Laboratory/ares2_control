#include <rclcpp/rclcpp.hpp>
#include "ares2_control/JoystickInterface.hpp"
#include "ares2_control/LocoController.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Create a MultiThreadedExecutor
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Initialize the joystick interface node
    auto joystick_interface = std::make_shared<ares2_control::JoystickInterface>();
    RCLCPP_INFO(joystick_interface->get_logger(), "JoystickInterface node initialized.");

    // Initialize the locomotion controller node
    auto loco_controller = std::make_shared<ares2_control::LocoController>();
    RCLCPP_INFO(loco_controller->get_logger(), "LocoController node initialized.");

    // Add nodes to the executor
    executor->add_node(joystick_interface);
    executor->add_node(loco_controller);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting executor to spin nodes...");
    
    // Spin the executor
    executor->spin();

    // Shutdown and clean up
    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("main"), "ROS 2 shutdown complete.");
    return 0;
}
