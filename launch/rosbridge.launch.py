import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rosbridge_launch_file = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch_file)
        )
    ])
