#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

MAX_DGRAM = 65000        # keep under single-packet UDP size

class UdpImageRelay(Node):
    """ROS 2 → UDP one-way image bridge."""
    def __init__(self):
        super().__init__('udp_image_relay')

        # ── tunables exposed as ROS 2 params ────────────────────────────
        self.declare_parameter('ip',    '192.168.199.100')
        self.declare_parameter('port',  5005)
        self.declare_parameter('qual',  70)      # JPEG quality 0-100
        dst_ip  = self.get_parameter('ip').get_parameter_value().string_value
        dst_port= self.get_parameter('port').get_parameter_value().integer_value
        self.jpg_q = int(self.get_parameter('qual').get_parameter_value().integer_value)
        self.socket_addr = (dst_ip, dst_port)

        # ── setup helpers ───────────────────────────────────────────────
        self.sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.bridge = CvBridge()

        self.create_subscription(Image,
                                 'drone0/camera/image',
                                 self.on_image,
                                 qos_profile_sensor_data)
        self.get_logger().info(f'Streaming to {dst_ip}:{dst_port} (JPEG q={self.jpg_q})')

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            ok, enc = cv2.imencode('.jpg', frame,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), self.jpg_q])
            if not ok:
                return  # skip faulty frame
            if enc.nbytes > MAX_DGRAM:
                self.get_logger().warn('Frame too big for one UDP datagram – dropped.')
                return
            self.sock.sendto(enc, self.socket_addr)
        except Exception as e:
            self.get_logger().error(str(e))

def main():
    rclpy.init()
    rclpy.spin(UdpImageRelay())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
