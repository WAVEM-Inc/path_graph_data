from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    package_name: str = "path_storage"
    executable_name: str = "path_storage_manager"

    path_storage_manager = Node(
        package=package_name,
        executable=executable_name,
        name=executable_name,
        output="screen",
        parameters=[],
        arguments=["--ros-args", "--log-level", "info"],
    )
    # create and return launch description object
    return LaunchDescription(
        [
            path_storage_manager,
        ]
    )
