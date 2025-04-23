#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class AltitudeControlledVelocityPublisher(Node):
    def __init__(self):
        super().__init__('altitude_controlled_velocity_publisher')

        # Publisher for MAVROS setpoint velocity
        self.publisher_ = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        # QoS profile for the pose subscriber (Best Effort)
        qos_profile_pose = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscribe to the local position to get altitude (using the z coordinate)
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile_pose
        )

        # Subscriber for external velocity commands
        self.input_velocity_subscriber = self.create_subscription(
            Twist,
            '/input_velocity',
            self.input_velocity_callback,
            10
        )

        # Timer to publish setpoints at 20 Hz (0.05 sec interval)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Drone state variables
        self.current_altitude = 0.0    # This is the z-coordinate altitude
        self.target_altitude = 10.0    # Target altitude in meters
        self.altitude_reached = False  # Flag to indicate the target altitude has been reached

        # Variables for storing external velocity inputs
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0
        self.new_input_received = False

        self.get_logger().info("🚁 AltitudeControlledVelocityPublisher started.")

    def pose_callback(self, msg):
        # Update the current altitude (from z coordinate)
        self.current_altitude = msg.pose.position.z
        # Check if the altitude threshold is reached (using the z coordinate)
        if not self.altitude_reached and self.current_altitude >= self.target_altitude:
            self.altitude_reached = True
            self.get_logger().info(f"✅ Altitude reached: {self.current_altitude:.2f} m")

    def input_velocity_callback(self, msg):
        # Store velocity commands provided from external input
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vz = msg.linear.z
        self.yaw_rate = msg.angular.z
        self.new_input_received = True
        self.get_logger().info(
            f"📥 Received input velocity: x={self.vx}, y={self.vy}, z={self.vz}, yaw_rate={self.yaw_rate}"
        )

    def timer_callback(self):
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        # Optionally, set the frame_id, e.g., "base_link"
        # vel_msg.header.frame_id = "base_link"

        if not self.altitude_reached:
            # While current altitude (z) is below 10 m, command a climb (z velocity = 0.5 m/s)
            vel_msg.twist.linear.z = 0.5
            vel_msg.twist.linear.x = 0.0
            vel_msg.twist.linear.y = 0.0
            vel_msg.twist.angular.z = 0.0
            self.get_logger().info(f"🔼 Climbing... Altitude: {self.current_altitude:.2f} m (z coordinate)")
        else:
            # Once altitude (z) has reached 10 m, use external input if provided
            if self.new_input_received:
                vel_msg.twist.linear.x = self.vx
                vel_msg.twist.linear.y = self.vy
                vel_msg.twist.linear.z = self.vz  # This now allows vertical control if desired
                vel_msg.twist.angular.z = self.yaw_rate
                self.get_logger().info(
                    f"🚀 Publishing input velocity: x={self.vx}, y={self.vy}, z={self.vz}, yaw_rate={self.yaw_rate}"
                )
                self.new_input_received = False  # Reset after consuming input command
            else:
                # Otherwise, command a hover (all velocities zero)
                vel_msg.twist.linear.x = 0.0
                vel_msg.twist.linear.y = 0.0
                vel_msg.twist.linear.z = 0.0
                vel_msg.twist.angular.z = 0.0
                self.get_logger().info("🛑 No input; hovering at the target altitude.")

        # Publish the velocity setpoint
        self.publisher_.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AltitudeControlledVelocityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
