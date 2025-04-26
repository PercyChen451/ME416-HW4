#!/usr/bin/env python
"""
A node that subscribes to images from the camera,
runs the color-segmentation algorithm, and publishes the centroid
of the segmented image
"""

import cv2
import image_processing as ip
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImagePublisher(Node):
    '''
    A publish-upon-message node for flipping images in a stream
    '''
    def __init__(self):
        super().__init__('image_segment')
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 1)
        self.pub_image = self.create_publisher(Image, '/image/segmented', 10)
        self.pub_centroid = self.create_publisher(PointStamped,
                                                  '/image/centroid', 10)
        self.bridge = CvBridge()

    def camera_callback(self, msg):
        '''
        Get  images from topic, flip, and republish
        '''
        img = self.bridge.imgmsg_to_cv2(msg)
        bound_low, bound_high = ip.classifier_parameters()
        img_segmented = ip.image_segment(img, bound_low, bound_high)
        # compute centroid
        x_centroid = ip.image_centroid_horizontal(img_segmented)
        # add the line on the segmented image
        img_segmented_line = ip.image_line_vertical(
            ip.image_one_to_three_channels(img_segmented), x_centroid)
        # publish segmented image to topic /image/segmented
        self.pub_image.publish(
            self.bridge.cv2_to_imgmsg(img_segmented_line, encoding='rgb8'))

        # prepare centroid stamped message
        msg = PointStamped()
        msg.point.x = float(x_centroid)  # PointStamped.point.x expects a float
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_centroid.publish(msg)


def main(args=None):
    '''
    Setup and run node
    '''
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
