#ifndef LOCO_CONTROLLER_HPP
#define LOCO_CONTROLLER_HPP

#include <cmath>
#include <mutex>
#include <atomic>
#include <tmotor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "ares2_msgs/msg/motor_command.hpp"
#include "ares2_msgs/msg/wheel_command_array.hpp"
#include "ares2_msgs/msg/motor_feedback.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>



#define WHEEL_RADIUS 0.1425f // meters
#define ROBOT_WIDTH 0.75f // meters
#define RAD_TO_DEG 57.2957795131f

namespace ares2_control
{
  class LocoController : public rclcpp::Node
  {
    public:
      LocoController() : Node("loco_controller"),
                         m_wheel_commands{0.0, 0.0, 0.0, 0.0},
                         m_motor_name_map{"front right", "front left", "rear right", "rear left"},
                         m_overheating(false)
      {
        int temp_limit(50);
        std::string control_method;
        std::string can_interface;
        std::vector<int64_t> wheel_ids;

        
        this->declare_parameter("locomotion/motor_temp_lim", 50);
        this->declare_parameter("locomotion/control_method", std::string("velocity"));
        this->declare_parameter("locomotion/control_degree", 1);
        this->declare_parameter("general/loop_rate", 100.0);
        this->declare_parameter("general/can_interface", std::string("vcan0"));
        this->declare_parameter("locomotion/wheel_ids", std::vector<int>{1, 2, 3, 4});



        RCLCPP_INFO(this->get_logger(), "Initializing parameters.");
        temp_limit = this->get_parameter("locomotion/motor_temp_lim").as_int();
        control_method = this->get_parameter("locomotion/control_method").as_string();
        m_control_degree = this->get_parameter("locomotion/control_degree").as_int();
        m_control_freq = this->get_parameter("general/loop_rate").as_double();
        can_interface = this->get_parameter("general/can_interface").as_string();
        wheel_ids = this->get_parameter("locomotion/wheel_ids").as_integer_array();
        
        
        m_maximum_temperature = std::max(INT8_MIN, std::min(INT8_MAX, temp_limit));
        m_can_interface = can_interface;
        for (size_t i = 0; i < 4; ++i) m_motor_ids[i] = wheel_ids[i];

        RCLCPP_INFO(this->get_logger(), "Control method: %s", control_method.c_str());
        RCLCPP_INFO(this->get_logger(), "CAN interface: %s", can_interface.c_str());
        RCLCPP_INFO(this->get_logger(),"Locomotion: Wheel IDs: %ld, %ld, %ld, %ld", wheel_ids[0], wheel_ids[1], wheel_ids[2], wheel_ids[3]);
        RCLCPP_INFO(this->get_logger(),"Locomotion: Control loop rate: %f", m_control_freq);


        connectMotors();
        m_wheels_pub[0] = this->create_publisher<ares2_msgs::msg::MotorFeedback>("front_right/feedback", 1);
        m_wheels_pub[1] = this->create_publisher<ares2_msgs::msg::MotorFeedback>("front_left/feedback", 1);
        m_wheels_pub[2] = this->create_publisher<ares2_msgs::msg::MotorFeedback>("rear_right/feedback", 1);
        m_wheels_pub[3] = this->create_publisher<ares2_msgs::msg::MotorFeedback>("rear_left/feedback", 1);

        m_wheel_vel[0] = this->create_publisher<std_msgs::msg::Float64>("front_right/velocity", 1);
        m_wheel_vel[1] = this->create_publisher<std_msgs::msg::Float64>("front_left/velocity", 1);
        m_wheel_vel[2] = this->create_publisher<std_msgs::msg::Float64>("rear_right/velocity", 1);
        m_wheel_vel[3] = this->create_publisher<std_msgs::msg::Float64>("rear_left/velocity", 1);

      
        
        if (control_method == "velocity")
        {
          RCLCPP_INFO(this->get_logger(), "Enacting VELOCITY control.");
          m_twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 1, std::bind(&LocoController::robotTwistVelocityCallback, this, std::placeholders::_1)
          );
          m_wheels_cmd_sub = this->create_subscription<ares2_msgs::msg::WheelCommandArray>(
            "cmd_arr", 1, std::bind(&LocoController::wheelArrayVelocityCallback, this, std::placeholders::_1)
          );
          m_control_timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / m_control_freq)),
            std::bind(&LocoController::velocityControlLoop, this)
          );
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "Enacting TORQUE control.");
          m_twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&LocoController::robotTwistTorqueCallback, this, std::placeholders::_1)
          );
          m_wheels_cmd_sub = this->create_subscription<ares2_msgs::msg::WheelCommandArray>(
            "cmd_arr", 10, std::bind(&LocoController::wheelArrayTorqueCallback, this, std::placeholders::_1)
          );
          m_control_timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / m_control_freq)),
            std::bind(&LocoController::torqueControlLoop, this)
          );
        }
      }

      
        
        
      

    private:
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_twist_sub;
      rclcpp::Subscription<ares2_msgs::msg::WheelCommandArray>::SharedPtr m_wheels_cmd_sub;
      rclcpp::Publisher<ares2_msgs::msg::MotorFeedback>::SharedPtr m_wheels_pub[4];
      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_wheel_vel[4];
      TMotor::AKManager m_motor_array[4];
      uint8_t m_motor_ids[4];
      std::string m_can_interface;
      float m_wheel_commands[4];
      double m_control_freq;
      int m_control_degree;
      std::mutex m_control_mutex;
      rclcpp::TimerBase::SharedPtr m_control_timer;
      std::atomic<bool> m_overheating;
      std::vector<std::string> m_motor_name_map;
      int8_t m_maximum_temperature;

      void velocityControlLoop()
      {
        float cmds[4];
        bool overheating = isOverheating();

        for (size_t i = 0; i < 4; i++)
        {
          auto feedback = ares2_msgs::msg::MotorFeedback();
          feedback.position = m_motor_array[i].getPosition();
          feedback.velocity = m_motor_array[i].getVelocity();
          feedback.current = m_motor_array[i].getCurrent();
          feedback.temperature = m_motor_array[i].getTemperature();
          feedback.motor_fault = m_motor_array[i].getFault();

          std::lock_guard<std::mutex> guard(m_control_mutex);
          cmds[i] = m_wheel_commands[i];
          m_wheel_commands[i] *= 0.95;

          if (!overheating)
          {
            try
            {
              
              m_motor_array[i].sendVelocity(cmds[i]);
            }
            catch (TMotor::CANSocketException &e)
            {
              connectMotors();
            }
          }
          std_msgs::msg::Float64 velocity_msg;
          velocity_msg.data = cmds[i];
          m_wheel_vel[i]->publish(velocity_msg);
          m_wheels_pub[i]->publish(feedback);
        }
      }

      void torqueControlLoop()
      {
        float cmds[4];
        bool overheating = isOverheating();

        for (size_t i = 0; i < 4; i++)
        {
          auto feedback = ares2_msgs::msg::MotorFeedback();
          feedback.position = m_motor_array[i].getPosition();
          feedback.velocity = m_motor_array[i].getVelocity();
          feedback.current = m_motor_array[i].getCurrent();
          feedback.temperature = m_motor_array[i].getTemperature();
          feedback.motor_fault = m_motor_array[i].getFault();

          std::lock_guard<std::mutex> guard(m_control_mutex);
          cmds[i] = m_wheel_commands[i];
          m_wheel_commands[i] *= 0.95;

          if (!overheating)
          {
            try
            {
              m_motor_array[i].sendCurrent(cmds[i]);
            }
            catch (TMotor::CANSocketException &e)
            {
              connectMotors();
            }
          }
          std_msgs::msg::Float64 velocity_msg;
          velocity_msg.data = cmds[i];
          m_wheel_vel[i]->publish(velocity_msg);
          m_wheels_pub[i]->publish(feedback);
        }
      }

      void robotTwistVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
      {
        float linear = msg->linear.x;
        float angular = msg->angular.z;

        std::lock_guard<std::mutex> guard(m_control_mutex);
        float v_l = ((linear - angular * ROBOT_WIDTH) * WHEEL_RADIUS * RAD_TO_DEG - m_wheel_commands[0]*m_control_degree)/(m_control_degree + 1);
        float v_r = ((linear + angular * ROBOT_WIDTH) * WHEEL_RADIUS * RAD_TO_DEG + m_wheel_commands[1]*m_control_degree)/(m_control_degree + 1);      
        m_wheel_commands[0] = m_wheel_commands[2] = -v_r;       
        m_wheel_commands[1] = m_wheel_commands[3] = v_l;   
      }

      void wheelArrayVelocityCallback(const ares2_msgs::msg::WheelCommandArray::SharedPtr msg)
      {
        if (msg->data.size() != 4) return;
        for (const auto &command : msg->data) if (command.type != 2) return;

        std::lock_guard<std::mutex> guard(m_control_mutex);
        for (size_t i = 0; i < 4; ++i) m_wheel_commands[i] = msg->data[i].velocity;
      }

      void robotTwistTorqueCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
      {
        // Similar to robotTwistVelocityCallback but with torque commands
        float linear = msg->linear.x;
        float angular = msg->angular.z;

        std::lock_guard<std::mutex> guard(m_control_mutex);
        float v_l = ((linear - angular * ROBOT_WIDTH) * WHEEL_RADIUS * RAD_TO_DEG);
        float v_r = ((linear + angular * ROBOT_WIDTH) * WHEEL_RADIUS * RAD_TO_DEG);
        m_wheel_commands[0] = m_wheel_commands[2] = -v_r;       
        m_wheel_commands[1] = m_wheel_commands[3] = v_l; 
      }

      void wheelArrayTorqueCallback(const ares2_msgs::msg::WheelCommandArray::SharedPtr msg)
      {
        if (msg->data.size() != 4) return;
        for (const auto &command : msg->data) if (command.type != 1) return;

        std::lock_guard<std::mutex> guard(m_control_mutex);
        for (size_t i = 0; i < 4; ++i) m_wheel_commands[i] = msg->data[i].velocity;
      }

      void connectMotors()
      {
        RCLCPP_INFO(this->get_logger(), "Connecting to motors through the CAN interface.");
        for (size_t i = 0; i < 4; i++)
        {
          try
          {
            m_motor_array[i].setMotorID(m_motor_ids[i]);
            m_motor_array[i].connect(m_can_interface.c_str());
          }
          catch(TMotor::CANSocketException& e)
          {
            RCLCPP_WARN(this->get_logger(), "Motor %d at %s unable to reconnect: %s", m_motor_array[i].getMotorID(), m_motor_name_map[i].c_str(), e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            i--; // retry
          }
        }
      }

      bool isOverheating()
      {
        bool flag = false;
        for (int i = 0; i < 4; i++)
        {
          if (m_motor_array[i].getTemperature() > m_maximum_temperature)
          {
            RCLCPP_ERROR(this->get_logger(), "Motor %d: %s is overheating, shutting down locomotion.", i, m_motor_name_map[i].c_str());
            flag = true;
          }
        }
        m_overheating.store(flag);
        return flag;
      }
  };
}

#endif
