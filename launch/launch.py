from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rmcs_localization",
            executable="rmcs_localization",
            parameters=[
                [
                    FindPackageShare("rmcs_localization"),
                    "/config",
                    "/localization.yaml",
                ],
                {
                    "map_path": [
                        FindPackageShare("rmcs_localization"),
                        "/map/tcd-train.pcd",
                    ],
                },
            ],
            output="screen",
            respawn=True,
            respawn_delay=1.0,
        )
    ])
