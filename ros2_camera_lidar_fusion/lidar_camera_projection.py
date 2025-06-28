#!/usr/bin/env python3
"""ROS 2 node that projects LiDAR points onto a camera image and colour‑codes
those points by range (near → far).  Green dots are gone; every dot now uses a
per‑point colour taken from an OpenCV colour‑map (JET by default).
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import cv2
import numpy as np
from geometry_msgs.msg import PointStamped
import yaml
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

import tf2_ros
# import tf_transformations as tft

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from ros2_camera_lidar_fusion.read_yaml import extract_configuration
from std_msgs.msg import String, Bool, Float32



def rot_matrix_from_quat(qx, qy, qz, qw):
    # optional: normalise to be safe
    n = qx*qx + qy*qy + qz*qz + qw*qw
    if n == 0.0:
        return np.eye(3)
    s = 2.0 / n          # common scale factor

    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz

    return np.array([
        [1 - s*(yy + zz),     s*(xy - wz),       s*(xz + wy)],
        [    s*(xy + wz),  1 - s*(xx + zz),      s*(yz - wx)],
        [    s*(xz - wy),     s*(yz + wx),   1 - s*(xx + yy)]
    ])

# ────────────────────────────── helpers ──────────────────────────────────────

def load_extrinsic_matrix(yaml_path: str) -> np.ndarray:
    """Load a 4 × 4 homogeneous transform matrix from *yaml_path*."""
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"No extrinsic file found: {yaml_path}")

    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    if "extrinsic_matrix" not in data:
        raise KeyError(f"YAML {yaml_path} has no 'extrinsic_matrix' key.")

    T = np.asarray(data["extrinsic_matrix"], dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError("Extrinsic matrix is not 4x4.")
    return T


def load_camera_calibration(yaml_path: str) -> tuple[np.ndarray, np.ndarray]:
    """Return (K, distCoeffs) from a camera‑info YAML file produced by Kalibr / ROS."""
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"No camera calibration file: {yaml_path}")

    with open(yaml_path, "r") as f:
        calib = yaml.safe_load(f)

    K = np.asarray(calib["camera_matrix"]["data"], dtype=np.float64)
    dist = np.asarray(calib["distortion_coefficients"]["data"], dtype=np.float64).reshape(1, -1)
    return K, dist


def pointcloud2_to_xyz_array_fast(cloud_msg: PointCloud2, skip_rate: int = 1) -> np.ndarray:
    """Convert *cloud_msg* into an *N × 3* XYZ array.  Uses numpy for speed."""

    if cloud_msg.height == 0 or cloud_msg.width == 0:
        return np.zeros((0, 3), dtype=np.float32)

    field_names = [f.name for f in cloud_msg.fields]
    if not all(k in field_names for k in ("x", "y", "z")):
        return np.zeros((0, 3), dtype=np.float32)

    # Build dtype matching the point step (assumes x,y,z are float32)
    dtype = np.dtype([
        ("x", np.float32),
        ("y", np.float32),
        ("z", np.float32),
        ("_", "V{}".format(cloud_msg.point_step - 12)),
    ])

    raw = np.frombuffer(cloud_msg.data, dtype=dtype)
    pts = np.zeros((raw.shape[0], 3), dtype=np.float32)
    pts[:, 0] = raw["x"]
    pts[:, 1] = raw["y"]
    pts[:, 2] = raw["z"]

    if skip_rate > 1:
        pts = pts[::skip_rate]
    return pts


# ───────────────────────────── node class ────────────────────────────────────

class LidarCameraProjectionNode(Node):
    """Projects LiDAR points into the camera plane and colours them by range."""

    def __init__(self):
        super().__init__("lidar_camera_projection_node")

        # ── load YAML configuration ------------------------------------------------
        cfg = extract_configuration()
        if cfg is None:
            self.get_logger().error("Failed to extract configuration file.")
            return

        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        cfg_folder = cfg["general"]["config_folder"]

        #   ─ extrinsic
        extrinsic_yaml = os.path.join(cfg_folder, cfg["general"]["camera_extrinsic_calibration"])
        self.T_lidar_to_cam = load_extrinsic_matrix(extrinsic_yaml)

        #   ─ camera intrinsics
        cam_yaml = os.path.join(cfg_folder, cfg["general"]["camera_intrinsic_calibration"])
        self.camera_matrix, self.dist_coeffs = load_camera_calibration(cam_yaml)

        #   log
        self.get_logger().info(f"Loaded extrinsic\n{self.T_lidar_to_cam}")
        self.get_logger().info(f"Camera matrix\n{self.camera_matrix}")

        # ── topics -----------------------------------------------------------------
        lidar_topic = cfg["lidar"]["lidar_topic"]
        image_topic = cfg["camera"]["image_topic"]
        projected_topic = cfg["camera"]["projected_topic"]

        self.get_logger().info(f"Subscribing to LiDAR : {lidar_topic}")
        self.get_logger().info(f"Subscribing to camera: {image_topic}")

        self.image_sub = Subscriber(self, Image,         image_topic, qos_profile=q)
        self.lidar_sub = Subscriber(self, PointCloud2,    lidar_topic, qos_profile=q)
        self.odom_sub = self.create_subscription(Odometry,'/dlio/odom_node/odom', self.odom_callback,q)

        self.sub_center = self.create_subscription(String,'/person_center',self.center_callback,q)
        self.sub_confidence = self.create_subscription(Bool,'/person_confident',self.confidence_callback,q)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sync = ApproximateTimeSynchronizer([self.image_sub, self.lidar_sub], queue_size=5, slop=0.07)
        self.sync.registerCallback(self.sync_callback)
        self.confident = False
        self.last_center = None

        self.pub_image = self.create_publisher(Image, projected_topic, 1)
        self.distance_pub = self.create_publisher(PointStamped, '/person_point', 1)
        self.marker_pub = self.create_publisher(Marker, '/person_marker', 1)
        self.person_map_pub = self.create_publisher(PointStamped, '/person_point_map', 1)
        self.bridge    = CvBridge()

        # ── parameters -------------------------------------------------------------
        self.skip_rate = 1           # subsample LiDAR for speed
        self.d_min     = 0.0         # metres (colour map near‑end)
        self.d_max     = 25.0        # metres (colour map far‑end)
        self.nearest_px_threshold = 10.0
    # ─────────────────────────────────────────────────────────────────────────────
    def confidence_callback(self, msg: Bool):
        self.get_logger().info(f"Received person_confident: {msg.data}")
        self.confident = msg.data
    # ─────────────────────────────────────────────────────────────────────────────
    def center_callback(self, msg: String):
        data = msg.data
        if data and data != 'None':
            try:
                # split the comma-separated string
                x_str, y_str = data.split(',')
                x = float(x_str)
                y = float(y_str)
                self.get_logger().info(f"Parsed center → x: {x:.2f}, y: {y:.2f}")
                
                # TODO: do something with x, y
                # e.g., store them as attributes, trigger logic, etc.
                self.last_center = (x, y)

            except ValueError:
                self.get_logger().error(f"Malformed center data: '{data}'")
                self.last_center = None
        else:
            self.get_logger().info("Received center: None")
            self.last_center = None
    # ─────────────────────────────────────────────────────────────────────────────
    def odom_callback(self, msg: Odometry):
        """Extract translation (t) and rotation matrix (R) from Odometry."""
        self.current_odom = msg

        # ── translation vector t (odom → base) ────────────────────────────────────
        p = msg.pose.pose.position
        self.t_odom = np.array([p.x, p.y, p.z], dtype=np.float64)

        # ── rotation matrix R (odom → base) ───────────────────────────────────────
        q = msg.pose.pose.orientation
        self.R_odom = rot_matrix_from_quat(q.x, q.y, q.z, q.w)  # (3×3 numpy array)

        # optional: detailed debug
        self.get_logger().debug(
            f'Odom ⇒ t = [{self.t_odom[0]:.2f}, {self.t_odom[1]:.2f}, {self.t_odom[2]:.2f}] | '
            f'R row0 = {self.R_odom[0]}'
        )    
    # ─────────────────────────────────────────────────────────────────────────────
    def sync_callback(self, img_msg: Image, cloud_msg: PointCloud2):
        """Project LiDAR points into *img_msg* and republish the overlay."""

        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        # ── check TF transform: map → camera frame --------------------------------
        # try:
        #     self.trans = self.tf_buffer.lookup_transform(
        #         'map',                       # target frame
        #         img_msg.header.frame_id,     # usually "camera"
        #         rclpy.time.Time.from_msg(img_msg.header.stamp),
        #         rclpy.duration.Duration(seconds=0.1))          # timeout
        # except (tf2_ros.LookupException,
        #         tf2_ros.ExtrapolationException,
        #         tf2_ros.TransformException) as e:
        #     self.get_logger().warn(f'TF lookup failed: {e}')

        # q = self.trans.transform.rotation
        # R = rot_matrix_from_quat(q.x, q.y, q.z, q.w)[:3, :3]
        # t = np.array([self.trans.transform.translation.x,
        #             self.trans.transform.translation.y,
        #             self.trans.transform.translation.z])
        
        
         # ── check TF transform: map → camera frame --------------------------------
        # ── LiDAR → numpy array ----------------------------------------------------
        xyz_lidar = pointcloud2_to_xyz_array_fast(cloud_msg, skip_rate=self.skip_rate)
        if xyz_lidar.size == 0:
            self.get_logger().warn("Empty cloud; nothing to project.")
            self._publish(cv_img, img_msg.header)
            return

        # ── homogeneous transform: LiDAR → camera frame ---------------------------
        xyz_lidar_h = np.hstack((xyz_lidar.astype(np.float64), np.ones((xyz_lidar.shape[0], 1))))
        xyz_cam     = (xyz_lidar_h @ self.T_lidar_to_cam.T)[:, :3]  # drop w

        # keep only points in front of the camera (z > 0)
        mask = xyz_cam[:, 2] > 0.0
        xyz_cam_front = xyz_cam[mask]
        if xyz_cam_front.size == 0:
            self.get_logger().info("No points with z > 0.")
            self._publish(cv_img, img_msg.header)
            return

        # ── project 3‑D points to 2‑D ---------------------------------------------
        image_pts, _ = cv2.projectPoints(
            xyz_cam_front,
            np.zeros((3, 1)), np.zeros((3, 1)),  # rvec, tvec = identity (already in cam frame)
            self.camera_matrix,
            self.dist_coeffs,
        )
        image_pts = image_pts.reshape(-1, 2)

        # ── colour‑code by range ---------------------------------------------------
        dists = np.linalg.norm(xyz_cam_front, axis=1)
        d_clipped = np.clip(dists, self.d_min, self.d_max)
        indices = np.uint8(255 * (d_clipped - self.d_min) / (self.d_max - self.d_min))
        colours_bgr = cv2.applyColorMap(indices.reshape(-1, 1), cv2.COLORMAP_JET)[:, 0, :]

        # ── draw -------------------------------------------------------------------
        h, w = cv_img.shape[:2]
        for (u, v), col in zip(image_pts, colours_bgr):
            u_i, v_i = int(round(u)), int(round(v))
            if 0 <= u_i < w and 0 <= v_i < h:
                cv2.circle(cv_img, (u_i, v_i), 2, (int(col[0]), int(col[1]), int(col[2])), -1)
        # ── Distance query ---------------------------------------------------------
        
        if self.confident and self.last_center is not None:
        
            target_px = np.asarray(self.last_center, dtype=np.float64)
            d_pix2 = np.sum((image_pts - target_px) ** 2, axis=1)
            nearest_idx = int(np.argmin(d_pix2))
            pixel_error = float(np.sqrt(d_pix2[nearest_idx]))

            if pixel_error <= self.nearest_px_threshold:
                # distance_m = float(dists[nearest_idx])
                x, y, z = xyz_cam_front[nearest_idx]

                self.R_base_cam = np.array([
                    [ 0.0,  0.0,  1.0],
                    [-1.0,  0.0,  0.0],
                    [ 0.0, -1.0,  0.0]
                ], dtype=np.float64)

                ### New added
                cam_pt = np.array([x, y, z])   
                R_map_cam = self.R_odom @ self.R_base_cam
                map_pt = R_map_cam @ cam_pt + self.t_odom
                map_msg = PointStamped()
                map_msg.header.frame_id = 'map'
                map_msg.header    = img_msg.header
                map_msg.point.x, map_msg.point.y, map_msg.point.z = map_pt
                self.person_map_pub.publish(map_msg)
                




                ###
                pt_msg = PointStamped()
                pt_msg.header = img_msg.header       # keep timestamp + frame_id
                pt_msg.point.x = float(x)
                pt_msg.point.y = float(y)
                pt_msg.point.z = float(z)
                self.distance_pub.publish(pt_msg)
                marker = Marker()
                marker.header = img_msg.header        # copy time stamp
                marker.header.frame_id = 'camera'
                marker.ns = 'person'
                marker.id = 0
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.scale.x = marker.scale.y = marker.scale.z = 0.15

                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                marker.pose.position.x = float(x)
                marker.pose.position.y = float(y)
                marker.pose.position.z = float(z)
                

                marker.pose.orientation.w = 1.0       # neutral rotation

                self.marker_pub.publish(marker)

                self.get_logger().info(
                    f'Pixel ({target_px[0]:.1f},{target_px[1]:.1f}) → '
                    f'point ({x:.2f},{y:.2f},{z:.2f}) m  '
                    f'(pixel error {pixel_error:.1f})'
                )
            else:
                self.get_logger().debug(
                    f"Nearest LiDAR point is {pixel_error:.1f}px away – above threshold; ignoring."
                )

        # ── publish ----------------------------------------------------------------
        self._publish(cv_img, img_msg.header)

    # ─────────────────────────────────────────────────────────────────────────────
    def _publish(self, cv_img: np.ndarray, header):
        msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        msg.header = header
        self.pub_image.publish(msg)


# ─────────────────────────────── main ─────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
