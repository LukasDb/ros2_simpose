import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

# [x] subscribe to RGBD
# [ ] fix subscription
# [ ] add parameter to load specific weights
# [ ] add parameter to set to specific object
# [ ] load yolo weights
# [ ] load PVN3D weights
# [ ] yolo inference
# [ ] PVN3D inference
# [ ] pose estimation


class SimposeNode(Node):
    def __init__(self) -> None:
        super().__init__("simpose_node")
        self.pose_publisher = self.create_publisher(PoseStamped, "/simpose/estimated_pose", 10)

        # Subscribe to the depth and color image topics
        self.depth_subscriber = self.create_subscription(
            Image, "/camera/depth/image_rect_raw", self.depth_callback, 10
        )
        self.rgb_subscriber = self.create_subscription(
            Image, "/camera/color/image_raw", self.rgb_callback, 10
        )
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.camera_info_callback, 10
        )

    def depth_callback(self, msg: Image) -> None:
        # Convert the ROS Image message to a NumPy array
        depth_image: np.ndarray = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width
        )
        # Now depth_image contains the depth image
        self.get_logger().info(f"Received depth image: {depth_image.shape}")

    def rgb_callback(self, msg: Image) -> None:
        # Convert the ROS Image message to a NumPy array
        rgb_image: np.ndarray = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1
        )
        # Now rgb_image contains the RGB image
        self.get_logger().info(f"Received RGB image: {rgb_image.shape}")

    def camera_info_callback(self, msg: CameraInfo) -> None:
        # The intrinsic camera matrix is available in the CameraInfo message
        K: np.ndarray = np.array(msg.K).reshape((3, 3))
        # Now K contains the intrinsic matrix
        self.get_logger().info(f"Received camera info: {K}")

    def publish_pose(self) -> None:
        pose_msg: PoseStamped = PoseStamped()
        # Set the pose values here
        pose_msg.pose.position.x = 1.0
        pose_msg.pose.position.y = 2.0
        pose_msg.pose.position.z = 3.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.pose_publisher.publish(pose_msg)
        self.get_logger().info("Published pose")


def main(args: list = None) -> None:
    rclpy.init(args=args)
    simpose_node: SimposeNode = SimposeNode()
    rclpy.spin(simpose_node)
    simpose_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
