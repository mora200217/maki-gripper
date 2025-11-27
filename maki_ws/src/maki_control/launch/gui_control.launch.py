from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        
        # --- NODO MAKI CONTROL ---
        Node(
            package='maki_control',
            executable='maki_serial_bridge',
            name='maki_control',
            output='screen',
            parameters=[{
                "esp32_ip": "10.234.6.32"
            }]
        ),

        # --- NODO MAKI GUI ---
        Node(
            package='maki_gui',
            executable='maki_gui',
            name='maki_gui',
            output='screen'
        )
    ])
