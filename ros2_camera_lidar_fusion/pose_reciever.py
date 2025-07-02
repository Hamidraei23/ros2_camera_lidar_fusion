#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message
import socket

class PoseReceiver(Node):
    def __init__(self, udp_ip: str, udp_port: int):
        super().__init__('pose_receiver')
        # Optional: re-publish on a ROS 2 topic
        self.pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

        # Create & bind the UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((udp_ip, udp_port))
        self.get_logger().info(f"Listening for UDP PoseStamped on {udp_ip}:{udp_port}")

    def listen(self):
        while rclpy.ok():
            # Wait for the next message (blocks)
            data, addr = self.sock.recvfrom(65536)  # buffer size
            try:
                # Reconstruct the PoseStamped message
                msg = deserialize_message(data, PoseStamped)
            except Exception as e:
                self.get_logger().error(f"Failed to deserialize from {addr}: {e}")
                continue

            # Log and (re-)publish
            p = msg.pose.position
            o = msg.pose.orientation
            self.get_logger().info(
                f"Received from {addr} → "
                f"pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}), "
                f"ori=({o.x:.2f}, {o.y:.2f}, {o.z:.2f}, {o.w:.2f})"
            )
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # Bind to all interfaces on port 9999; change as needed
    udp_ip = '0.0.0.0'
    udp_port = 9999

    node = PoseReceiver(udp_ip, udp_port)
    try:
        node.listen()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
