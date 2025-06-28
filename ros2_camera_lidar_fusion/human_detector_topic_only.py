#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ultralytics import YOLO
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from collections import deque

class ThermalPersonDetector(Node):
    def __init__(self):
        super().__init__('yolo_person_center_publisher')

        # QoS: match camera publisher (best-effort reliability)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Publishers: center coordinates, annotated image, and confidence flag
        self.pub_center     = self.create_publisher(String, 'person_center', qos)
        self.pub_image      = self.create_publisher(Image,  'person_detection_image', qos)
        self.pub_confidence = self.create_publisher(Bool,   'person_confident', qos)

        # CV Bridge and YOLO model
        self.bridge = CvBridge()
        self.model = YOLO('yolo11n.pt')
        self.person_class_id = next((i for i,n in self.model.names.items() if n=='person'), 0)

        # Detection history for confidence
        self.detections = deque(maxlen=5)

        # Subscribe to drone camera image with matching QoS
        self.create_subscription(
            Image,
            '/drone0/camera/image',
            self.image_callback,
            qos
        )

        self.get_logger().info('YOLO Thermal Person Detector (ROS2) started listening on /drone0/camera/image')

    def image_callback(self, msg: Image):
        # Convert ROS Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO detection
        results = self.model(frame)[0]
        person_boxes = [b for b in results.boxes if int(b.cls.cpu().item()) == self.person_class_id]

        # Compute 2D center
        center_msg = String()
        if person_boxes:
            best = max(person_boxes, key=lambda b: float(b.conf.cpu().item()))
            x1, y1, x2, y2 = best.xyxy.cpu().numpy()[0]
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            center_msg.data = f"{cx:.2f},{cy:.2f}"
            self.detections.append(True)
        else:
            center_msg.data = 'None'
            self.detections.append(False)

        # Publish center
        self.pub_center.publish(center_msg)

        if len(self.detections) > 8 and all(self.detections[-5:]) and not self.once:
            gurant = True
        else:
            gurant = False

        # Compute and publish confidence: True if all of last 5 frames saw a person
        conf_msg = Bool()
        conf_msg.data = len(self.detections) == self.detections.maxlen and all(self.detections)
        self.pub_confidence.publish(conf_msg)

        # Annotate and publish image
        for b in person_boxes:
            x1, y1, x2, y2 = b.xyxy.cpu().numpy()[0].astype(int)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        out_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub_image.publish(out_img)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalPersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
