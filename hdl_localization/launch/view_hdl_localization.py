import os

from launch import LaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    current_dir = os.path.dirname(os.path.realpath(__file__))
    hdl_localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(current_dir, "hdl_localization.py"))
    )

    rviz_config_file = os.path.join(current_dir, "..", "rviz", "config.rviz")
    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    ld = LaunchDescription()
    ld.add_action(hdl_localization_node)
    ld.add_action(rviz_node)

    return ld
