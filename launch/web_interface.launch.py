from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "orion_web_interface"
    
    return LaunchDescription([
        Node(
            package=package_name,
            executable='backend',
            name='backend',
            output='screen'
        )
    ])
