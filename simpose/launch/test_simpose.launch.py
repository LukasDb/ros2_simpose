from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    ld = LaunchDescription()

    # start rviz2
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     output="screen",
    #     # arguments=['-d', 'src/simpose/simpose/config/simpose.rviz']
    # )
    # ld.add_action(rviz_node)

    # add simpose node
    simpose_node = Node(package="simpose", executable="simpose_node", output="screen")
    ld.add_action(simpose_node)

    # Start Intel RealSense node
    realsense_node = Node(
        package="realsense2_camera",
        namespace="realsense",
        name="realsense_node",
        executable="realsense2_camera_node",
        output="screen",
        parameters=[
            {
                "temporal_filter.enable": True,
                "spatial_filter.enable": True,
                "enable_sync": False,  # Enable depth and color stream synchronization
                "enable_rgbd": False,
                "enable_infra": False,
                "enable_infra1": False,
                "enable_infra1": False,
                "align_depth.enable": True,  # Enable depth alignment with RGB
                "rgb_camera.profile": "1920x1080x15",
            }
        ],
    )
    ld.add_action(realsense_node)

    return ld
