#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import subprocess
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry


class BagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # 1) Subscribe to original image stream
        self.subscription = self.create_subscription(Image,'/drone0/camera/image', self.callback_image,qos_profile=q)

        self.subscription = self.create_subscription(Odometry, '/dlio/odom_node/odom', self.callback_odom, qos_profile=q)

        self.subscription  # prevent unused variable warning

        # 2) Publisher for throttled 2 Hz stream
        self.pub_2hz = self.create_publisher(Image, '/image_2hz', qos_profile=q)
        self.pub_odom_2hz = self.create_publisher(Odometry, '/odom_2hz', qos_profile=q)

        # store latest image
        self.latest_img = None

        # timer fires every 0.5 s → 2 Hz
        self.create_timer(0.5, self.timer_publish)

        # 3) start ros2 bag recording in background
        subprocess.Popen([
            'ros2', 'bag', 'record',
            '/image_2hz', '/odom_2hz',
        ])
    def callback_odom(self, msg: Odometry):

        self.latest_odom = msg

    def callback_image(self, msg: Image):
        # stash the most recent image
        self.latest_img = msg

    def timer_publish(self):
        if self.latest_img is not None:
            # re-publish at 2 Hz
            self.pub_2hz.publish(self.latest_img)
            self.pub_odom_2hz.publish(self.latest_odom)
            self.get_logger().debug('Published one image at 2 Hz')
            self.get_logger().debug('Published one frame at 2 Hz')

def main(args=None):
    rclpy.init(args=args)
    node = BagRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
