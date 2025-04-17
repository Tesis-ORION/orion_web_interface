from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    package_name = "orion_web_interface"

    input_topic_1 = DeclareLaunchArgument("input_topic1", default_value="/camera_os30a/image_raw")
    output_topic_1 = DeclareLaunchArgument("output_topic1", default_value="/apc/left/image_base64")

    input_topic_2 = DeclareLaunchArgument("input_topic2", default_value="/camera_os30a/depth_image")
    output_topic_2 = DeclareLaunchArgument("output_topic2", default_value="/apc/depth/image_base64")

    # Nuevo parámetro para indicar el tipo de formato
    format_type_1 = DeclareLaunchArgument("format_type1", default_value="rgb")
    format_type_2 = DeclareLaunchArgument("format_type2", default_value="depth")

    backend_node1 = Node(
        package=package_name,
        executable="backend",
        name="backend_image",
        output="screen",
        parameters=[
            {"input_topic": LaunchConfiguration("input_topic1")},
            {"output_topic": LaunchConfiguration("output_topic1")},
            {"format_type": LaunchConfiguration("format_type1")}
        ]
    )

    backend_node2 = Node(
        package=package_name,
        executable="backend",
        name="backend_depth",
        output="screen",
        parameters=[
            {"input_topic": LaunchConfiguration("input_topic2")},
            {"output_topic": LaunchConfiguration("output_topic2")},
            {"format_type": LaunchConfiguration("format_type2")}
        ]
    )

    return LaunchDescription([
        input_topic_1, output_topic_1,
        input_topic_2, output_topic_2,
        format_type_1, format_type_2,
        backend_node1, backend_node2
    ])