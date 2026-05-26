#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

import cv2


class ImageCompressor(Node):

    def __init__(self):
        super().__init__('image_compressor')

        self.bridge = CvBridge()

        # Subscribe to raw image topic
        self.subscription = self.create_subscription(
            Image,
            '/mr_pinchy/payload/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish compressed image
        self.publisher = self.create_publisher(
            CompressedImage,
            '/mr_pinchy/payload/camera/image_compressed',
            10
        )

        self.get_logger().info('Image compressor node started')

    def image_callback(self, msg):
        try:
            # Convert ROS Image -> OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize to width=388, height=504
            resized = cv2.resize(frame, (388, 504))

            # Compress as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            success, encoded_image = cv2.imencode(
                '.jpg',
                resized,
                encode_param
            )

            if not success:
                self.get_logger().error('Failed to compress image')
                return

            # Create CompressedImage message
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = 'jpeg'
            compressed_msg.data = encoded_image.tobytes()

            # Publish
            self.publisher.publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = ImageCompressor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()