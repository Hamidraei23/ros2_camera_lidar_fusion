#!/usr/bin/env python3

import socket
import json
import threading
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist

class UDPToRos(Node):
    def __init__(self, listen_ip='0.0.0.0', listen_port=9999):
        super().__init__('udp_to_ros')

        # ROS QoS (matching your teleop publisher if needed)
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        # Latest velocities
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # Set up UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((listen_ip, listen_port))
        self.sock.setblocking(False)

        self.get_logger().info(f"Listening for UDP on {listen_ip}:{listen_port}")

        # Start receiver thread
        threading.Thread(target=self.udp_listener, daemon=True).start()

        # Publish at 10 Hz
        self.create_timer(0.1, self.publish_twist)

    def udp_listener(self):
        """Receive JSON packets over UDP and update speeds."""
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
            except BlockingIOError:
                time.sleep(0.01)
                continue

            try:
                payload = json.loads(data.decode('utf-8'))
                lin = float(payload.get('linear_x', 0.0))
                ang = float(payload.get('angular_z', 0.0))
                self.linear_speed = lin
                self.angular_speed = ang
                self.get_logger().debug(
                    f"Got UDP from {addr}: linear={lin:.2f}, angular={ang:.2f}"
                )
            except (ValueError, json.JSONDecodeError) as e:
                self.get_logger().warn(f"Failed to parse UDP packet: {e}")

    def publish_twist(self):
        """Publish the current velocities as a Twist message."""
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # Allow override of listen address/port via command-line
    ip = sys.argv[1] if len(sys.argv) > 1 else '0.0.0.0'
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 9999

    node = UDPToRos(listen_ip=ip, listen_port=port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down UDP→ROS bridge")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
