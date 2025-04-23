#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy

class CollisionDetectionNode(Node):
    def __init__(self):
        super().__init__('collision_detection_node')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Kalman filter parameters for acceleration
        self.accel_est = None       # Estimated acceleration (m/s^2)
        self.P_a = 1.0              # Estimation error covariance
        self.Q_a = 0.1              # Process noise variance for acceleration
        self.R_a = 0.5              # Measurement noise variance for acceleration
        self.accel_threshold = 1.5  # Residual threshold to flag collision for accel

        # Kalman filter parameters for velocity
        self.vel_est = None         # Estimated velocity (m/s)
        self.P_v = 1.0              # Estimation error covariance
        self.Q_v = 0.1              # Process noise variance for velocity
        self.R_v = 0.3              # Measurement noise variance for velocity
        self.vel_threshold = 0.8   # Residual threshold to flag collision for velocity

        self.last_imu_time = None   # To compute dt for acceleration filter
        self.last_odom_time = None  # To compute dt for velocity filter
        
        self.last_collision_time = 0.0  # Debounce collisions (in seconds)

        # Current measured velocity (calculated from odometry)
        self.current_velocity = 0.0

        # Subscriptions
        self.create_subscription(Imu, '/mavros/imu/data_raw', self.imu_cb, sensor_qos)
        self.create_subscription(Odometry, '/mavros/odometry/in', self.odom_cb, sensor_qos)

        # Publisher
        self.collision_pub = self.create_publisher(Bool, '/collision', QoSProfile(depth=10))

    def imu_cb(self, msg: Imu):
        # Compute acceleration magnitude from IMU (raw data may include gravity)
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        accel_meas = math.sqrt(ax**2 + ay**2 + az**2)

        # Get current time (seconds)
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        dt = 0.0
        if self.last_imu_time is None:
            dt = 0.1  # initial dt assumption
            self.accel_est = accel_meas  # initialize filter state
            self.P_a = 1.0
        else:
            dt = current_time - self.last_imu_time
            if dt <= 0.0:
                dt = 0.1
        self.last_imu_time = current_time

        # Kalman Filter Prediction for acceleration:
        # Constant model: x_pred = x; error increases by Q*dt.
        x_pred = self.accel_est
        P_pred = self.P_a + self.Q_a * dt

        # Measurement residual:
        residual_a = accel_meas - x_pred

        # Kalman Gain for acceleration:
        K_a = P_pred / (P_pred + self.R_a)

        # Update acceleration estimate:
        self.accel_est = x_pred + K_a * residual_a
        self.P_a = (1 - K_a) * P_pred

        self.get_logger().debug(f"Accel meas: {accel_meas:.2f}, est: {self.accel_est:.2f}, resid: {residual_a:.2f}")

        # If the residual exceeds threshold, flag a potential collision from acceleration:
        accel_collision = abs(residual_a) > self.accel_threshold

        # Check collision condition combining both accel and velocity; if available.
        if hasattr(self, 'vel_collision'):
            collision_flag = accel_collision and self.vel_collision
        else:
            collision_flag = accel_collision  # fallback if velocity not yet updated

        if collision_flag:
            self.publish_collision()

    def odom_cb(self, msg: Odometry):
        # Compute full 3D velocity magnitude
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        vel_meas = math.sqrt(vx**2 + vy**2 + vz**2)
        self.current_velocity = vel_meas

        # Get current time (seconds)
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        dt = 0.0
        if self.last_odom_time is None:
            dt = 0.1
            self.vel_est = vel_meas  # initialize state
            self.P_v = 1.0
        else:
            dt = current_time - self.last_odom_time
            if dt <= 0.0:
                dt = 0.1
        self.last_odom_time = current_time

        # Kalman Filter Prediction for velocity:
        v_pred = self.vel_est
        P_pred = self.P_v + self.Q_v * dt

        residual_v = vel_meas - v_pred

        K_v = P_pred / (P_pred + self.R_v)
        self.vel_est = v_pred + K_v * residual_v
        self.P_v = (1 - K_v) * P_pred

        self.get_logger().debug(f"Vel meas: {vel_meas:.2f}, est: {self.vel_est:.2f}, resid: {residual_v:.2f}")

        # If the velocity residual exceeds threshold, flag potential collision from velocity
        self.vel_collision = abs(residual_v) > self.vel_threshold

        # Optionally, you might decide to trigger collision here immediately if no accel update is present,
        # but our imu_cb checks for both signals.

    def publish_collision(self):
        # Publish collision only if sufficient debounce time (e.g., 0.5 seconds) has passed.
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.last_collision_time > 0.5:
            self.last_collision_time = current_time
            msg = Bool()
            msg.data = True
            self.collision_pub.publish(msg)
            self.get_logger().warn("Collision detected! Published True on /collision")

def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
