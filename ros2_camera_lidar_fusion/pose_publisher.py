#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import os

class PosePublisher(Node):
    def __init__(self, yaml_path: str):
        super().__init__('pose_publisher')
        self.pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

        # Load YAML file
        if not os.path.isfile(yaml_path):
            self.get_logger().error(f"YAML file not found: {yaml_path}")
            rclpy.shutdown()
            return

        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        # Parse into list of PoseStamped
        self.poses = []
        for entry in data.get('poses', []):
            ps = PoseStamped()
            # copy header
            ps.header.frame_id = entry['header'].get('frame_id', 'map')
            # Note: you could set ps.header.stamp = self.get_clock().now().to_msg() when publishing
            # copy position
            pos = entry['pose']['position']
            ps.pose.position.x = pos['x']
            ps.pose.position.y = pos['y']
            ps.pose.position.z = pos['z']
            # copy orientation
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

        self.get_logger().info(f"Loaded {len(self.poses)} poses. Press Enter to publish each one.")

    def run(self):
        idx = 0
        while rclpy.ok() and idx < len(self.poses):
            input(f"\nPress Enter to publish pose #{idx + 1}/{len(self.poses)}...")
            # update timestamp
            self.poses[idx].header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.poses[idx])
            self.get_logger().info(f"Published pose #{idx + 1}")
            idx += 1

        self.get_logger().info("All poses published, shutting down.")
        rclpy.shutdown()


def main(args=None):
    import sys
    # if len(sys.argv) < 2:
    #     print("Usage: publish_poses.py <path_to_yaml>")
    #     return

    yaml_file = "/home/hami/workspaces/raicam_ws/src/ros2_camera_lidar_fusion/config/poses.yaml"
    rclpy.init(args=args)
    node = PosePublisher(yaml_file)
    # Only enter the loop if initialization succeeded
    if node.poses:
        node.run()


if __name__ == '__main__':
    main()
