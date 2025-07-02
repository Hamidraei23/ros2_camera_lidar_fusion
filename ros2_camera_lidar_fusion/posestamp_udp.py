#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from geometry_msgs.msg import PoseStamped
import yaml
import os
import socket

class PosePublisher(Node):
    def __init__(self, yaml_path: str, udp_ip: str, udp_port: int):
        super().__init__('pose_publisher')
        # (Optional) still publish on ROS2 topic if you want:
        # self.pub = self.create_publisher(PoseStamped, 'pose_stamped', 10)

        # Set up UDP socket
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info(f"UDP socket -> {udp_ip}:{udp_port}")

        # Load YAML file as before
        if not os.path.isfile(yaml_path):
            self.get_logger().error(f"YAML file not found: {yaml_path}")
            rclpy.shutdown()
            return

        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        self.poses = []
        for entry in data.get('poses', []):
            ps = PoseStamped()
            ps.header.frame_id = entry['header'].get('frame_id', 'map')
            pos = entry['pose']['position']
            ps.pose.position.x = pos['x']
            ps.pose.position.y = pos['y']
            ps.pose.position.z = pos['z']
            ori = entry['pose']['orientation']
            ps.pose.orientation.x = ori['x']
            ps.pose.orientation.y = ori['y']
            ps.pose.orientation.z = ori['z']
            ps.pose.orientation.w = ori['w']
            self.poses.append(ps)

        if not self.poses:
            self.get_logger().warn("No poses found in the YAML file.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Loaded {len(self.poses)} poses. Press <Enter> to send each one over UDP.")

    def run(self):
        idx = 0
        while rclpy.ok() and idx < len(self.poses):
            input(f"\nPress Enter to send pose #{idx + 1}/{len(self.poses)}...")
            # stamp with “now”
            self.poses[idx].header.stamp = self.get_clock().now().to_msg()

            # --- Option A: send raw ROS2-serialized bytes ---
            data = serialize_message(self.poses[idx])
            self.sock.sendto(data, (self.udp_ip, self.udp_port))
            self.get_logger().info(f"Sent binary PoseStamped #{idx+1} ({len(data)} bytes)")

            # --- Option B: send JSON instead (uncomment if you prefer) ---
            # import json
            # from rclpy.serialization import message_to_ordereddict
            # d = message_to_ordereddict(self.poses[idx])
            # js = json.dumps(d).encode('utf-8')
            # self.sock.sendto(js, (self.udp_ip, self.udp_port))
            # self.get_logger().info(f"Sent JSON PoseStamped #{idx+1}")

            idx += 1

        self.get_logger().info("All poses sent, shutting down.")
        rclpy.shutdown()


def main(args=None):
    # Hard-coded paths & UDP target:
    yaml_file = "/home/hami/workspaces/raicam_ws/src/ros2_camera_lidar_fusion/config/poses.yaml"
    udp_ip   = "192.168.199.201"   # <— replace with your receiver’s IP
    udp_port = 9999              # <— replace with your receiver’s port

    rclpy.init(args=args)
    node = PosePublisher(yaml_file, udp_ip, udp_port)
    if node.poses:
        node.run()


if __name__ == '__main__':
    main()
