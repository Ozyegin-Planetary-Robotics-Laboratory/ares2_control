from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the joy node to handle joystick input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace='ares2_control',
            output='screen'
        ),

        # Launch the ares2_control_node with inline parameters
        Node(
            package='ares2_control',
            executable='ares2_control_node',  # Replace with the actual name of your executable
            name='ares2_control',
            namespace='ares2_control',
            output='screen',
            parameters=[{
                'locomotion.speed.angular_scale': 500.0,
                'locomotion.speed.linear_scale': 300.0,
                'locomotion.motor_temp_lim': 70,
                'locomotion.control_degree': 30,
                'locomotion.control_method': 'velocity',
                'locomotion.wheel_ids': [3, 4, 2, 1],
                'occupancy_map.resolution': 5,
                'occupancy_map.length': 50,
                'general.loop_rate': 20,
                'general.can_interface': 'vcan0'
            }]  # Parameters are passed inline here
        )
    ])
