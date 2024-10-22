#ifndef JOYSTICK_INTERFACE_HPP
#define JOYSTICK_INTERFACE_HPP

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace ares2_control
{
  class JoystickInterface : public rclcpp::Node
  {
    public:
      JoystickInterface() : Node("joystick_interface"),
        m_linear_scale(300.0),
        m_angular_scale(500.0),
        m_loop_rate(30.0)
      {
        // Declare and retrieve parameters
        this->declare_parameter<double>("locomotion/speed/linear_scale", 0.0);
        this->declare_parameter<double>("locomotion/speed/angular_scale", 0.0);
        this->declare_parameter<double>("general/loop_rate", 30.0);

        //this->get_parameter("locomotion/speed/linear_scale", m_linear_scale);
        //this->get_parameter("locomotion/speed/angular_scale", m_angular_scale);
        //this->get_parameter("general/loop_rate", m_loop_rate);

        // Subscription to joystick messages
        m_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, 
                    std::bind(&JoystickInterface::joyCallback, this, std::placeholders::_1));
        
        // Publisher for velocity command messages
        m_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

        // Timer for control loop, using the specified loop rate
        m_control_timer = this->create_wall_timer(
          std::chrono::milliseconds(static_cast<int>(1000 / m_loop_rate)), 
          std::bind(&JoystickInterface::controlLoop, this)
        );
      }

    private:
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_sub;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub;
      rclcpp::TimerBase::SharedPtr m_control_timer;  // Timer for the control loop
      std::mutex m_command_mutex;
      double m_linear_scale, m_angular_scale, m_loop_rate;
      geometry_msgs::msg::Twist m_twist;

      // Callback for joystick input
      void joyCallback(const std::shared_ptr<sensor_msgs::msg::Joy> joy)
      {
        std::lock_guard<std::mutex> lock(m_command_mutex);
        m_twist.linear.x = joy->axes[1] * m_linear_scale;
        m_twist.angular.z = joy->axes[0] * m_angular_scale;
        RCLCPP_INFO(this->get_logger(), "axes[0]: %f, axes[1]: %f", joy->axes[0], joy->axes[1]);
      }

      // Control loop, called by the timer
      void controlLoop()
      {
        auto twist = std::make_shared<geometry_msgs::msg::Twist>();
        {
          std::lock_guard<std::mutex> lock(m_command_mutex);
          twist->linear.x = m_twist.linear.x *= 0.9875;  // Gradually reduce velocity
          twist->angular.z = m_twist.angular.z *= 0.9875;
        }
        m_pub->publish(*twist);
      }

  }; // JoystickInterface
}; // namespace ares_control

#endif // JOYSTICK_INTERFACE_HPP
