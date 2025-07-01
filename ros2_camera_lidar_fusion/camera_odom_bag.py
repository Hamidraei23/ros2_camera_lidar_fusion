#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
import subprocess
import re

class BagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribe to raw image stream
        self.image_sub = self.create_subscription(
            Image,
            '/drone0/camera/image',
            self.callback_image,
            qos_profile=q
        )

        # Subscribe to raw odometry stream
        self.odom_sub = self.create_subscription(
            Odometry,
            '/dlio/odom_node/odom',
            self.callback_odom,
            qos_profile=q
        )

        # Publishers for throttled 2 Hz streams
        self.image_pub = self.create_publisher(Image, '/image_2hz', qos_profile=q)
        self.odom_pub = self.create_publisher(Odometry, '/odom_2hz', qos_profile=q)

        # Buffers for latest messages
        self.latest_img = None
        self.latest_odom = None

        # Timer for 2 Hz publishing (0.5 s)
        self.create_timer(0.5, self.timer_publish)

        # Start ros2 bag recording both throttled topics
        self.proc = subprocess.Popen(
            [
                'ros2', 'bag', 'record',
                '/image_2hz',
                '/odom_2hz',
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )

        # Parse recorder output to log save directory
        while True:
            line = self.proc.stdout.readline()
            if not line:
                break
            m = re.search(r'Writing to directory: (.+)$', line)
            if m:
                bag_path = m.group(1)
                self.get_logger().info(f'Bag is being saved in: {bag_path}')
                break

    def callback_image(self, msg: Image):
        self.latest_img = msg

    def callback_odom(self, msg: Odometry):
        self.latest_odom = msg

    def timer_publish(self):
        if self.latest_img is not None:
            self.image_pub.publish(self.latest_img)
            self.get_logger().debug('Published image at 2 Hz')
        if self.latest_odom is not None:
            self.odom_pub.publish(self.latest_odom)
            self.get_logger().debug('Published odometry at 2 Hz')

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
