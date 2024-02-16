import rclpy
import cv2
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import numpy as np
import cv_bridge

# [x] subscribe to RGBD
# [x] fix subscription
# [x] correct realsense parameters
# [ ] parametrize topics
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
        # self.depth_subscriber = self.create_subscription(
        #     CompressedImage,
        #     "/realsense/aligned_depth_to_color/image_raw/compressedDepth",
        #     self.depth_callback,
        #     10,
        # )
        self.rgb_subscriber = self.create_subscription(
            CompressedImage,
            "/realsense/aligned_depth_to_color/image_raw/compressed",
            self.rgb_callback,
            10,
        )
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            "/realsense/aligned_depth_to_color/camera_info",
            self.camera_info_callback,
            10,
        )

    def depth_callback(self, msg: CompressedImage) -> None:
        # Convert the ROS Image message to a NumPy array
        return
        depth_image: np.ndarray = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width
        )
        # Now depth_image contains the depth image
        self.get_logger().info(f"Received depth image: {depth_image.shape}")

    def rgb_callback(self, msg: CompressedImage) -> None:
        # Convert the ROS Image message to a NumPy array
        # np_arr = np.fromstring(msg.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        self.get_logger().error(f"Received RGB image: {msg}")
        rgb_image = cv_bridge.CvBridge().compressed_imgmsg_to_cv2(msg)

        # rgb_image: np.ndarray = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        #     msg.height, msg.width, -1
        # )
        # # Now rgb_image contains the RGB image
        self.get_logger().info(f"Received RGB image: {rgb_image.shape}")

    def camera_info_callback(self, msg: CameraInfo) -> None:
        # The intrinsic camera matrix is available in the CameraInfo message
        K: np.ndarray = np.array(msg.k).reshape((3, 3))
        # Now K contains the intrinsic matrix
        self.get_logger().info(f"Received camera info: {K.shape}")

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
