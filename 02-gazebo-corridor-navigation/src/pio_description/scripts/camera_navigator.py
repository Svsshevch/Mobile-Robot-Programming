#!/usr/bin/env python3

from pathlib import Path

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String


class CameraNavigator(Node):
    def __init__(self):
        super().__init__('camera_navigator')

        #folder where images will be saved
        self.image_folder = Path.home() / 'ros2_ws/src/pio_description/camera_images'
        self.image_folder.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_image = None
        self.image_counter = 0

        #subscribe to RGB camera images
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        #subscribe to turn messages from navigation_node.py
        self.turn_event_sub = self.create_subscription(
            String,
            '/mrp/turn_event',
            self.turn_event_callback,
            10
        )

        self.get_logger().info('Camera navigator started')
        self.get_logger().info(f'Images will be saved in: {self.image_folder}')
        self.get_logger().info('Waiting for /mrp/turn_event messages...')

    def image_callback(self, msg):
        #turn events will trigger saving the latest image at that moment.
        self.latest_image = msg

    def turn_event_callback(self, msg):
        # This function is called when navigation_node.py starts a turn.
        turn_name = msg.data

        self.get_logger().info(f'Received turn event: {turn_name}')

        self.save_image(turn_name)

    def save_image(self, turn_name):
        if self.latest_image is None:
            self.get_logger().warn(
                'Turn happened, but no camera image has arrived yet.'
            )
            return

        try:
            #convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(
                self.latest_image,
                desired_encoding='bgr8'
            )

            #use camera timestamp in the filename
            stamp = self.latest_image.header.stamp

            filename = self.image_folder / (
                f'{self.image_counter:03d}_{turn_name}_'
                f'{stamp.sec}_{stamp.nanosec}.png'
            )

            success = cv2.imwrite(str(filename), cv_image)

            if success:
                self.get_logger().info(f'Saved image: {filename}')
                self.image_counter += 1
            else:
                self.get_logger().error(f'Could not write image: {filename}')

        except Exception as e:
            self.get_logger().error(f'Could not save camera image: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = CameraNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()