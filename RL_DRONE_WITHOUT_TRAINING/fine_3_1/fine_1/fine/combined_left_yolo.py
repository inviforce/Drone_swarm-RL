#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from cv2.ximgproc import createDisparityWLSFilter, createRightMatcher
import time
import torch
import gc

class StereoYOLOFusion(Node):
    def __init__(self):
        super().__init__('stereo_yolo_fusion_node')
        self.bridge = CvBridge()
        self.left_images = {}
        self.right_images = {}

        # Memory management configuration
        self.cache_expiration = 2.0  # Reduced expiration time for faster cleanup
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        
        # Stereo vision parameters
        self.baseline = 0.06
        self.focal_px = 700

        # Initialize YOLO model with memory optimization
        self.model = YOLO('/home/c3ilab/Desktop/fine_3_1/fine_3_1/fine_1/fine/best.pt').to(self.device)
        self.model.fuse()
        self.model.eval()  # Set to evaluation mode for inference

        # Image processing components
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        
        # Stereo matchers configuration
        self.left_matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=96,
            blockSize=5,
            P1=8 * 5 ** 2,
            P2=32 * 5 ** 2,
            uniquenessRatio=10,
            speckleWindowSize=50,
            speckleRange=2,
            disp12MaxDiff=1,
            preFilterCap=31,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        self.right_matcher = createRightMatcher(self.left_matcher)
        self.wls = createDisparityWLSFilter(self.left_matcher)
        self.wls.setLambda(5000)
        self.wls.setSigmaColor(1.2)

        # ROS2 configuration
        self.left_sub = self.create_subscription(Image, '/camera/left/image', self.left_callback, 10)
        self.right_sub = self.create_subscription(Image, '/camera/right/image', self.right_callback, 10)
        self.publisher_ = self.create_publisher(Point, '/detected_objects', 10)

        # Timing control
        self.last_publish_time = time.time()
        self.publish_interval = 0.5

        self.get_logger().info(f"Initialized with {self.device.upper()} acceleration")

    def left_callback(self, msg):
        self._image_callback(msg, is_left=True)

    def right_callback(self, msg):
        self._image_callback(msg, is_left=False)

    def _image_callback(self, msg, is_left):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        receipt_time = time.time()
        target_dict = self.left_images if is_left else self.right_images
        target_dict[stamp] = (receipt_time, msg)
        self.try_process(stamp)
        self.clean_cache(receipt_time)

    def clean_cache(self, current_time):
        # Clean both caches with explicit memory release
        for img_dict in [self.left_images, self.right_images]:
            expired = [stamp for stamp, (t, _) in img_dict.items() 
                     if current_time - t > self.cache_expiration]
            for stamp in expired:
                del img_dict[stamp]
                # Force garbage collection for expired entries
                gc.collect()
                if self.device == 'cuda':
                    torch.cuda.empty_cache()

    def try_process(self, stamp):
        current_time = time.time()
        if current_time - self.last_publish_time < self.publish_interval:
            return

        if stamp in self.left_images and stamp in self.right_images:
            left_msg = self.left_images.pop(stamp)[1]
            right_msg = self.right_images.pop(stamp)[1]
            self.process_and_publish(left_msg, right_msg)
            self.last_publish_time = current_time

    def process_and_publish(self, left_msg, right_msg):
        # Convert ROS images to OpenCV with explicit memory management
        left_gray = self.bridge.imgmsg_to_cv2(left_msg, 'mono8')
        right_gray = self.bridge.imgmsg_to_cv2(right_msg, 'mono8')
        
        # Process images with CLAHE
        left_eq = self.clahe.apply(left_gray)
        right_eq = self.clahe.apply(right_gray)

        # Compute disparity map
        dispL = self.left_matcher.compute(left_eq, right_eq)
        dispR = self.right_matcher.compute(right_eq, left_eq)
        disp_wls = self.wls.filter(dispL, left_eq, None, dispR)
        disp_wls = np.float32(disp_wls) / 16.0

        # Calculate depth map with memory-efficient operations
        with np.errstate(divide='ignore', invalid='ignore'):
            depth_map = np.where(disp_wls > 0, 
                               (self.focal_px * self.baseline) / disp_wls, 
                               0.0).astype(np.float32)

        # YOLO inference with memory optimization
        left_color = self.bridge.imgmsg_to_cv2(left_msg, 'bgr8')
        with torch.no_grad():  # Disable gradient calculation
            results = self.model.predict(
                left_color,
                imgsz=640,
                conf=0.2,
                device=self.device,
                half=(self.device == 'cuda'),
                verbose=False
            )

        valid_signals = []
        for box in results[0].boxes:
            # Convert tensor to CPU memory first
            if box.xyxy.is_cuda:
                coords = box.xyxy.cpu().numpy()[0]
            else:
                coords = box.xyxy.numpy()[0]
            del box  # Explicit tensor deletion

            x1, y1, x2, y2 = map(int, coords)
            x1 = max(0, min(x1, depth_map.shape[1]-1))
            y1 = max(0, min(y1, depth_map.shape[0]-1))
            x2 = max(0, min(x2, depth_map.shape[1]-1))
            y2 = max(0, min(y2, depth_map.shape[0]-1))

            # Calculate depth with memory-efficient operations
            roi_depth = depth_map[y1:y2, x1:x2]
            valid_mask = roi_depth > 0
            if np.any(valid_mask):
                avg_depth = np.mean(roi_depth[valid_mask])
                signal = Point()
                signal.x = float((x1 + x2) // 2)
                signal.y = float((y1 + y2) // 2)
                signal.z = float(avg_depth)
                valid_signals.append(signal)

        # Explicit memory cleanup
        del left_gray, right_gray, left_eq, right_eq, dispL, dispR, disp_wls, depth_map, left_color
        if self.device == 'cuda':
            torch.cuda.empty_cache()
        gc.collect()

        # Publish results
        for signal in valid_signals[:3]:
            self.publisher_.publish(signal)
            self.get_logger().debug(
                f"Published: X={signal.x:.1f}, Y={signal.y:.1f}, Z={signal.z:.2f}m"
            )

def main(args=None):
    rclpy.init(args=args)
    node = StereoYOLOFusion()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

if __name__ == '__main__':
    main()