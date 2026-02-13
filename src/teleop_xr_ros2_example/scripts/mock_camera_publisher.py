#!/usr/bin/env python3
"""
Mock camera publisher for teleop_xr XR view testing.
Publishes multiple camera streams with different test patterns.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np
import cv2


class MockCameraPublisher(Node):
    """Publishes mock camera streams with test patterns."""

    def __init__(self):
        super().__init__("mock_camera_publisher")

        # Declare parameters
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)

        # Get parameters
        rate = self.get_parameter("publish_rate").value
        self.width = self.get_parameter("image_width").value
        self.height = self.get_parameter("image_height").value

        # Create publishers for different camera streams
        self.camera_publishers = {
            "head": self.create_publisher(Image, "/camera/head/image_raw", 10),
            "wrist_left": self.create_publisher(
                Image, "/camera/wrist_left/image_raw", 10
            ),
            "wrist_right": self.create_publisher(
                Image, "/camera/wrist_right/image_raw", 10
            ),
            "workspace": self.create_publisher(
                Image, "/camera/workspace/image_raw", 10
            ),
        }

        # Frame counters for each camera
        self.frame_counts = {key: 0 for key in self.camera_publishers.keys()}

        # Create timer
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info(
            f"Mock camera publisher started: {len(self.camera_publishers)} streams at {rate} Hz"
        )

    def generate_test_pattern(self, camera_name: str, frame_count: int) -> np.ndarray:
        """
        Generate a test pattern image for the specified camera.

        Args:
            camera_name: Name of the camera (head, wrist_left, wrist_right, workspace)
            frame_count: Current frame number for animation

        Returns:
            RGB image as numpy array (height, width, 3)
        """
        # Create base image
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Different patterns for different cameras
        if camera_name == "head":
            # Gradient pattern with moving circle
            for y in range(self.height):
                for x in range(self.width):
                    img[y, x] = [
                        int(255 * x / self.width),
                        int(255 * y / self.height),
                        128,
                    ]
            # Moving circle
            center_x = int(self.width / 2 + 100 * np.sin(frame_count * 0.05))
            center_y = int(self.height / 2 + 100 * np.cos(frame_count * 0.05))
            cv2.circle(img, (center_x, center_y), 30, (255, 255, 0), -1)

        elif camera_name == "wrist_left":
            # Checkerboard pattern with color shift
            square_size = 40
            shift = (frame_count // 10) % 2
            for y in range(0, self.height, square_size):
                for x in range(0, self.width, square_size):
                    if ((x // square_size + y // square_size + shift) % 2) == 0:
                        img[y : y + square_size, x : x + square_size] = [200, 50, 50]
                    else:
                        img[y : y + square_size, x : x + square_size] = [50, 50, 200]

        elif camera_name == "wrist_right":
            # Rotating bars
            angle = frame_count * 2
            for i in range(10):
                bar_y = int((i / 10) * self.height)
                color_shift = int(255 * np.sin(angle * 0.02 + i * 0.5))
                img[bar_y : bar_y + self.height // 10, :] = [
                    128 + color_shift // 2,
                    128 - color_shift // 2,
                    128,
                ]

        elif camera_name == "workspace":
            # Grid with animated text
            # Draw grid
            for i in range(0, self.width, 50):
                cv2.line(img, (i, 0), (i, self.height), (100, 100, 100), 1)
            for i in range(0, self.height, 50):
                cv2.line(img, (0, i), (self.width, i), (100, 100, 100), 1)
            # Animated background color
            bg_intensity = int(50 + 30 * np.sin(frame_count * 0.03))
            img = cv2.add(
                img,
                np.array(
                    [bg_intensity, bg_intensity // 2, bg_intensity // 3], dtype=np.uint8
                ),
            )

        # Add camera label
        font = cv2.FONT_HERSHEY_SIMPLEX
        label = f"{camera_name.upper()} - Frame {frame_count}"
        cv2.putText(img, label, (10, 30), font, 0.7, (255, 255, 255), 2)

        # Add FPS counter (approximate)
        fps_text = f"30 FPS"
        cv2.putText(
            img, fps_text, (10, self.height - 10), font, 0.5, (255, 255, 255), 1
        )

        return img

    def numpy_to_image_msg(self, img: np.ndarray, camera_name: str) -> Image:
        """
        Convert numpy array to ROS Image message.

        Args:
            img: RGB image as numpy array
            camera_name: Name of the camera for frame_id

        Returns:
            ROS Image message
        """
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{camera_name}_optical_frame"

        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()

        return msg

    def timer_callback(self):
        """Publish images for all cameras."""
        for camera_name, publisher in self.camera_publishers.items():
            # Generate test pattern
            frame_count = self.frame_counts[camera_name]
            img = self.generate_test_pattern(camera_name, frame_count)

            # Convert to ROS message
            msg = self.numpy_to_image_msg(img, camera_name)

            # Publish
            publisher.publish(msg)

            # Increment frame counter
            self.frame_counts[camera_name] += 1


def main(args=None):
    rclpy.init(args=args)
    node = MockCameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
