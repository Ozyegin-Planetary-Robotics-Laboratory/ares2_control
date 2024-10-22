# THIS CODE IS NOT TESTED YET.
# Ares2 Control System

## Overview

This project implements the control system for the Ares2 robot, including the **LocoController** (a motion controller that listens to velocity or torque commands) and the **JoystickInterface** (which maps joystick inputs to velocity commands). The system is designed to run on the actual robot using **tmotor** hardware.

## Prerequisites

Before building and running the project, ensure you have the following prerequisites installed:

### ROS 2 (Humble)
The project is built for **ROS 2 Humble**. Follow the instructions for installing ROS 2:
- [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)

### Dependencies
Install the required ROS 2 packages:
```bash
sudo apt update
sudo apt install ros-humble-rclcpp ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-std-msgs
```

### TMotor Library
You need the **tmotor** library for motor control.
1. Build the `tmotor` library as a ROS 2 package:
   - Clone the `tmotorcan_cpp` repository into your workspace.
   - Build it using colcon.
   ```bash
   colcon build --packages-select tmotorcan_cpp
   ```

2. Make sure the library is installed in `ros2_ws/install/tmotorcan_cpp/lib` and the header is in `ros2_ws/install/tmotorcan_cpp/include`.

### Ares2 Messages (ares2_msgs)
You will need the **ares2_msgs** package, which defines the necessary custom messages for this project:
1. Clone the **ares2_msgs** package into your workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/ares2_msgs.git
   ```

2. Build the package using colcon:
   ```bash
   colcon build --packages-select ares2_msgs
   ```

## Building the Project

1. Clone the **Ares2 Control System** into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/ares2_control.git
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Running the System

### Running on Actual Hardware

1. Bring up the CAN interface for motor communication:
   ```bash
   sudo ip link set can0 type can bitrate 500000
   sudo ip link set up can0
   ```

2. Run the control node:
   ```bash
   ros2 launch ares2_control launch.py
   ```

### Joystick Control

To control the robot using a joystick:
1. Ensure a joystick is connected to your machine.
2. Use the **JoystickInterface** to map joystick axes to the robot’s velocity control:
   ```bash
   ros2 topic echo /cmd_vel
   ```
   The `/cmd_vel` topic should publish velocity commands based on joystick input.

## Project Structure

```
ares2_control
├── CMakeLists.txt                # Build instructions     
├── package.xml
├── include
    ├── LocoController.hpp
│   └── JoystickInterface.hpp     # Header files for the controllers
├── launch                        # Launch files to run the system
    └── launch.py
└── src
    └── main.cpp                  # Node instantiating
                                 
```

---

This README provides the necessary steps to build and run the Ares2 control system on actual hardware. Ensure that all dependencies are installed, and follow the instructions to test the system with motors and a joystick.