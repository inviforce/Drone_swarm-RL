#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YOLOv11nObjectDetector(Node):
    def __init__(self):
        super().__init__('yolo')

        self.get_logger().info("Initializing YOLOv11n Object Detector Node...")

        # Load the YOLOv11n model
        try:
            self.model = YOLO('/home/c3ilab/Desktop/fine_3/fine_1/fine/best.pt')
            self.get_logger().info("YOLOv11n model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            raise e

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribe to the left camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/left/image',
            self.image_callback,
            10
        )

        self.get_logger().info("Subscribed to /stereo/left/image_raw")

    def image_callback(self, msg):
        self.get_logger().info("Received an image.")
        try:
            # Convert ROS2 Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info("Image converted to OpenCV format.")

            # Perform YOLOv11n detection
            results = self.model(cv_image)

            # Confidence threshold to filter detections
            confidence_threshold = 0.2  # Show detections with confidence >= 0.2
            detections = results[0].boxes  # Get the boxes from the result
            filtered_detections = []

            # Filter detections by confidence
            for i, box in enumerate(detections):
                confidence = box.conf[0].item()  # Get confidence score of the detection
                if confidence >= confidence_threshold:
                    filtered_detections.append(box)

            self.get_logger().info(f"Filtered {len(filtered_detections)} detections above the threshold.")

            # Visualize results
            annotated = results[0].plot()  # Annotated image with all detections
            self.get_logger().info("Annotated image ready.")

            # Show the annotated frame
            cv2.imshow("YOLOv11n Detection", annotated)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    print("ROS2 node initialization...")
    detector = YOLOv11nObjectDetector()
    print("Spinning ROS2 node...")
    rclpy.spin(detector)

    # Cleanup
    detector.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
    print("Node shutdown complete.")

if __name__ == '__main__':
    main()
