from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # start rviz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        # arguments=['-d', 'src/simpose/simpose/config/simpose.rviz']
    )
    ld.add_action(rviz_node)

    # add simpose node
    simpose_node = Node(package="simpose", executable="simpose_node", output="screen")
    ld.add_action(simpose_node)

    # Start Intel RealSense node
    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        output="screen",
        parameters=[
            {
                "filters": {
                    "spatial": {
                        "filter_magnitude": 2,
                        "filter_smooth_alpha": 0.5,
                        "filter_smooth_delta": 20,
                        "hole_filling_mode": 2,
                    },
                    "temporal": {"filter_smooth_alpha": 0.4, "filter_smooth_delta": 20},
                },
                "align_depth": True,  # Enable depth alignment with RGB
                "color_width": 1080,  # Set color width to 1080
                "color_height": 1920,  # Set color height to 1920
                "fps": 15,  # Set FPS to 15
            }
        ],
    )
    ld.add_action(realsense_node)

    return ld
