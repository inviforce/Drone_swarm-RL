#!/usr/bin/env python3
import os
import csv
import time
import math
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
from replay_buffer import ReplayBuffer
from policy import Actor, Critic

# Check for GPU availability
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

# Enhanced Hyperparameters
STATE_DIM = 24
ACTION_DIM = 3
BUFFER_SIZE = 200000
BATCH_SIZE = 256  # Increased batch size for better GPU utilization
GAMMA = 0.99
TAU = 0.005
LR_ACTOR = 1e-4
LR_CRITIC = 3e-4
NOISE_DECAY = 0.995
CHECKPOINT_PATH = "checkpoint.pth"
LOG_DIR = "training/logs"
REWARDS_CSV = os.path.join(LOG_DIR, "rewards.csv")

# Environment parameters
MIN_ALTITUDE = 10.0
MAX_ALTITUDE = 12.0
TARGET_ALTITUDE = (MIN_ALTITUDE + MAX_ALTITUDE) / 2
COLLISION_PENALTY = -500.0
BASE_WAYPOINT_THRESHOLD = 1.5
SAFETY_DISTANCE = 1.2
ALTITUDE_CORRECTIVE_VELOCITY = 0.5

class DDPGTrainer(Node):
    def __init__(self):
        super().__init__('ddpg_trainer_node')
        os.makedirs(LOG_DIR, exist_ok=True)
        
        # Initialize parameters
        self.declare_parameter('verbose_logging', True)
        self.last_print_time = time.time()
        self.last_action = None
        self.episode_start_time = time.time()
        self.prev_actions = []
        self.episode_reward = 0.0
        # Drone state
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)
        self.yaw = 0.0
        self.bboxes = []
        self.is_resetting = False
        self.reset_start_time = 0.0
        self.reset_duration = 60  # seconds
        # Training state
        self.episode_count = 0
        self.train_step = 0
        self.noise_scale = 1.0
        self.successful_episodes = 0
        self.collision_detected = False
        self.waypoints = self.load_waypoints()
        self.current_waypoint_idx = 0
        self.last_waypoint_time = time.time()

        # Networks with GPU support
        self.actor = Actor(STATE_DIM, ACTION_DIM).to(device)
        self.critic = Critic(STATE_DIM, ACTION_DIM).to(device)
        self.target_actor = Actor(STATE_DIM, ACTION_DIM).to(device)
        self.target_critic = Critic(STATE_DIM, ACTION_DIM).to(device)
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=LR_ACTOR)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=LR_CRITIC)
        self.replay_buffer = ReplayBuffer(BUFFER_SIZE)
        self.update_networks(tau=1.0)

        # ROS setup
        self.action_pub = self.create_publisher(Twist, '/input_velocity', 5)
        self.episode_done_pub = self.create_publisher(Bool, '/episode_done', 10)
        
        # Subscribers
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', 
                               self.velocity_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', 
                               self.position_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Point, '/detected_objects', self.detected_objects_callback, 10)
        self.create_subscription(Bool, '/collision', self.collision_callback, 10)

        self.load_checkpoint()
        self.training_timer = self.create_timer(0.1, self.training_loop)


    def velocity_callback(self, msg):
        self.velocity = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ])

    def position_callback(self, msg):
        self.position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        q = msg.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        
    def train(self):
        """Train neural networks using experience replay with GPU acceleration"""
        if len(self.replay_buffer) < BATCH_SIZE:
            return

        states, actions, rewards, next_states = self.replay_buffer.sample(BATCH_SIZE)
        
        # Move data to GPU
        states = torch.FloatTensor(states).to(device)
        actions = torch.FloatTensor(actions).to(device)
        rewards = torch.FloatTensor(rewards).unsqueeze(1).to(device)
        next_states = torch.FloatTensor(next_states).to(device)

        # Update critic
        with torch.no_grad():
            target_actions = self.target_actor(next_states)
            target_q = self.target_critic(next_states, target_actions)
            target_q = rewards + GAMMA * target_q

        current_q = self.critic(states, actions)
        critic_loss = nn.MSELoss()(current_q, target_q)
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        nn.utils.clip_grad_norm_(self.critic.parameters(), 1.0)
        self.critic_optimizer.step()

        # Update actor
        actor_loss = -self.critic(states, self.actor(states)).mean()
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # Update target networks
        self.update_networks()


    def reset_environment(self):
        """Reset Gazebo environment using external scripts"""
        killer_script = "/home/c3ilab/Desktop/fine_3_1/fine_3_1/fine_1/fine/killer.sh"
        boxes_script = "/home/c3ilab/Desktop/fine_3_1/fine_3_1/fine_1/fine/boxes.py"
        waypoint_script = "/home/c3ilab/Desktop/fine_3_1/fine_3_1/fine_1/fine/waypoint.py"

        try:
            # Conditionally generate new obstacles and waypoints
            if self.successful_episodes > 64 or self.episode_count > 512:
                subprocess.run(["python3", boxes_script], check=True)
                subprocess.run(["python3", waypoint_script], check=True)

            # Kill existing processes and restart
            subprocess.run(["ros2 daemon stop"], shell=True)
            subprocess.run(["ros2 daemon start"], shell=True)
            
            # Kill lingering processes
            subprocess.run(["pkill -f FastRTPS"], shell=True)
            subprocess.run(["pkill -f discovery"], shell=True)
            self.get_logger().info("Resetting environment...")
            subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{killer_script}; exec bash"])
            time.sleep(60)  # Increased wait time for full reset
            self.get_logger().info("Environment reset complete")

        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Environment reset failed: {str(e)}")


    def end_episode(self):
        """Handle episode termination"""
        # Clear velocity commands
        self.send_zero_velocity()
        
        # Perform reset sequence
        self.reset_environment()
        
        # Maintain reset state until duration passes
        self.episode_count += 1
        self.collision_detected = False
        self.episode_reward = 0.0
        self.current_waypoint_idx = 0
        # Update success counter
        if self.current_waypoint_idx >= len(self.waypoints)-1:
            self.successful_episodes += 1
        # Create CSV with header if not exists
        if not os.path.exists(REWARDS_CSV):
            with open(REWARDS_CSV, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['Episode', 'Step', 'Waypoints', 'Collision', 'Total_Reward'])

    # Save training data
        with open(REWARDS_CSV, 'a') as f:
            writer = csv.writer(f)  # ← Add this line
            writer.writerow([
                self.episode_count,
                self.train_step,
                self.current_waypoint_idx,
                int(self.collision_detected),
                self.episode_reward
            ])

        # Save model every 200 episodes
        if self.episode_count % 200 == 0:
            self.save_checkpoint()

        # Reset environment on collision
        if self.collision_detected:
            self.reset_environment()
            self.episode_reward = 0.0
        else:
            # Soft reset for non-collision cases
            self.current_waypoint_idx = 0
            self.position = np.zeros(3)
            self.bboxes = []
            self.waypoints = self.load_waypoints()

        # Reset episode tracking
        self.collision_detected = False
        self.episode_start_time = time.time()
        self.episode_done_pub.publish(Bool(data=True))
        self.get_logger().info(f"Episode {self.episode_count} ended")
        
    def detected_objects_callback(self, msg):
        self.bboxes.append(msg)

    def collision_callback(self, msg):
        self.collision_detected = msg.data

    def update_networks(self, tau=TAU):
        for target, source in zip([self.target_actor, self.target_critic],
                                [self.actor, self.critic]):
            for target_param, param in zip(target.parameters(), source.parameters()):
                target_param.data.copy_(tau * param.data + (1.0 - tau) * target_param.data)

    def load_waypoints(self):
        waypoints = []
        try:
            with open("/home/c3ilab/Desktop/fine_3/fine_1/waypoint.txt", "r") as f:
                for line in f:
                    parts = list(map(float, line.strip().split()))
                    if len(parts) >= 2:
                        waypoints.append(np.array([parts[0], parts[1], TARGET_ALTITUDE]))
        except:
            waypoints = [np.array([10, 0, TARGET_ALTITUDE]), 
                        np.array([10, 10, TARGET_ALTITUDE])]
        return waypoints

    def get_state(self):
        # Obstacle features
        obstacles = sorted(self.bboxes, key=lambda p: p.x**2 + p.y**2)[:5]
        obstacle_features = []
        for p in obstacles:
            obstacle_features.extend([p.x - self.position[0], p.y - self.position[1], p.z - self.position[2]])
        obstacle_features += [0.0]*(15 - len(obstacle_features))

        # Waypoint features
        if self.current_waypoint_idx < len(self.waypoints):
            target = self.waypoints[self.current_waypoint_idx]
            wp_rel = target[:2] - self.position[:2]
            wp_dist = np.linalg.norm(wp_rel)
            wp_dir = wp_rel / (wp_dist + 1e-8)
        else:
            wp_dir = np.zeros(2)
            wp_dist = 0.0

        # Velocity in body frame
        vx_body = self.velocity[0] * math.cos(self.yaw) + self.velocity[1] * math.sin(self.yaw)
        vy_body = -self.velocity[0] * math.sin(self.yaw) + self.velocity[1] * math.cos(self.yaw)

        return np.array([
            vx_body, vy_body, self.velocity[2],
            self.position[2],
            math.sin(self.yaw), math.cos(self.yaw),
            wp_dir[0], wp_dir[1], wp_dist,
            *obstacle_features
        ], dtype=np.float32)
    
    def load_checkpoint(self):
        """Load model weights with device mapping"""
        if os.path.exists(CHECKPOINT_PATH):
            try:
                checkpoint = torch.load(CHECKPOINT_PATH, map_location=device)
                self.actor.load_state_dict(checkpoint['actor'])
                self.critic.load_state_dict(checkpoint['critic'])
                self.target_actor.load_state_dict(checkpoint['target_actor'])
                self.target_critic.load_state_dict(checkpoint['target_critic'])
                
                # Load other training states
                self.episode_count = checkpoint.get('episode', 0)
                self.noise_scale = checkpoint.get('noise_scale', 1.0)
                self.successful_episodes = checkpoint.get('successes', 0)
                
                # Ensure models are on correct device
                self.actor.to(device)
                self.critic.to(device)
                self.target_actor.to(device)
                self.target_critic.to(device)
                
                self.get_logger().info(f"Loaded checkpoint from {CHECKPOINT_PATH}")
            except Exception as e:
                self.get_logger().error(f"Checkpoint loading error: {str(e)}")
        else:
            self.get_logger().info("No checkpoint found, initializing new models")

    def save_checkpoint(self):
        """Save current training state"""
        checkpoint = {
            'actor': self.actor.state_dict(),
            'critic': self.critic.state_dict(),
            'target_actor': self.target_actor.state_dict(),
            'target_critic': self.target_critic.state_dict(),
            'episode': self.episode_count,
            'noise_scale': self.noise_scale,
            'successes': self.successful_episodes
        }
        torch.save(checkpoint, CHECKPOINT_PATH)
        self.get_logger().info(f"Checkpoint saved at episode {self.episode_count}")

    @property
    def waypoint_threshold(self):
        """Dynamic waypoint acceptance threshold"""
        base = BASE_WAYPOINT_THRESHOLD
        reduction = 0.01 * self.successful_episodes
        return max(0.8, base - reduction)
    
    def compute_reward(self, distance):
        """Multi-faceted reward function with 8 components"""
        if self.collision_detected:
            return COLLISION_PENALTY * (0.97 ** self.episode_count)
        # 1. Waypoint progress reward (continuous)
        prev_waypoint = self.waypoints[max(0, self.current_waypoint_idx-1)]
        dist_to_prev = np.linalg.norm(self.position[:2] - prev_waypoint[:2])
        progress_reward = 2.5 * (dist_to_prev - distance)  # Reward distance reduction
        
        # 2. Waypoint achievement bonus
        waypoint_bonus = 0.0
        if distance < self.waypoint_threshold:
            waypoint_bonus = 50.0 * (1 + self.current_waypoint_idx/len(self.waypoints))
        
        # 3. Obstacle avoidance (non-linear danger zone)
        obstacle_penalty = 0.0
        for p in self.bboxes[:3]:  # Consider closest 3 obstacles
            obj_dist = np.linalg.norm([p.x-self.position[0], p.y-self.position[1]])
            if obj_dist < SAFETY_DISTANCE * 2:
                obstacle_penalty -= (SAFETY_DISTANCE * 2 / (obj_dist + 0.1)) ** 2
        
        # 4. Altitude management (piecewise penalty)
        alt_error = abs(self.position[2] - TARGET_ALTITUDE)
        if alt_error > 1.0:
            altitude_penalty = -3.0 * alt_error**2
        else:
            altitude_penalty = -1.5 * alt_error
        
        # 5. Directional efficiency (velocity projection)
        if self.current_waypoint_idx < len(self.waypoints):
            target_dir = (self.waypoints[self.current_waypoint_idx][:2] - self.position[:2])
            target_dir /= np.linalg.norm(target_dir) + 1e-8
            vel_projection = np.dot(self.velocity[:2], target_dir)
            directional_reward = 0.8 * vel_projection
        
        # 6. Action smoothness (acceleration penalty)
        smoothness_penalty = 0.0
        if len(self.prev_actions) >= 2:
            accel = np.linalg.norm(np.array(self.prev_actions[-1]) - 
                            np.array(self.prev_actions[-2]))
            smoothness_penalty = -0.3 * accel**2
        
        # 7. Time penalty (encourage efficiency)
        time_penalty = -0.05 * (time.time() - self.last_waypoint_time)
        
        # 8. Completion bonus
        completion_bonus = 200.0 if self.current_waypoint_idx >= len(self.waypoints)-1 else 0.0

        total_reward = (
            progress_reward +
            waypoint_bonus +
            directional_reward +
            completion_bonus +
            obstacle_penalty +
            altitude_penalty +
            smoothness_penalty +
            time_penalty
        )
        
        return np.clip(total_reward, -100.0, 150.0)

    def training_loop(self):
        if self.is_resetting:
            # Calculate remaining reset time
            elapsed = time.time() - self.reset_start_time
            if elapsed < self.reset_duration:
                # Send zero velocity and skip training
                self.send_zero_velocity()
                return
            else:
                # Finish reset sequence
                self.is_resetting = False
                self.get_logger().info("Resume training after reset")
                self.episode_start_time = time.time()
                return
            
        if self.collision_detected or time.time() - self.episode_start_time > 300:
            self.initiate_reset()
            return
        
        # Get current state and convert to tensor
        state = self.get_state()
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(device)

        # Generate action with GPU acceleration
        with torch.no_grad():
            action = self.actor(state_tensor).squeeze(0).cpu().numpy()

        # Adaptive exploration with decaying noise
        noise = self.noise_scale * np.random.normal(0, 0.2, ACTION_DIM)
        action = np.clip(action + noise, -1.0, 1.0)
        self.prev_actions.append(action.copy())
        if len(self.prev_actions) > 5:  # Keep last 5 actions for smoothness calculation
            self.prev_actions.pop(0)

        # Altitude control layer
        current_z = self.position[2]
        altitude_error = current_z - TARGET_ALTITUDE
        if abs(altitude_error) > 0.5:  # Only intervene when error exceeds 0.5m
            action[1] = np.clip(-altitude_error * 0.4, -1.0, 1.0)

        # Collision avoidance override
        if len(self.bboxes) > 0:
            closest_obj = min(self.bboxes, key=lambda p: np.linalg.norm([p.x, p.y, p.z]))
            obj_distance = np.linalg.norm([closest_obj.x, closest_obj.y, closest_obj.z])
            if obj_distance < SAFETY_DISTANCE * 1.5:
                # Calculate evasion vector
                evade_dir = np.array([self.position[0] - closest_obj.x,
                                    self.position[1] - closest_obj.y,
                                    0])  # Maintain altitude
                evade_dir = evade_dir / (np.linalg.norm(evade_dir) + 1e-8)
                action[0] = np.clip(evade_dir[0], -1.0, 1.0)
                action[2] = np.clip(evade_dir[1] * 0.5, -1.0, 1.0)

        # Publish action
        twist = Twist()
        twist.linear.x = float(action[0])
        twist.linear.z = float(action[1])
        twist.angular.z = float(action[2])
        self.action_pub.publish(twist)
        self.last_action = action

        # Waypoint updates
        current_waypoint = self.waypoints[self.current_waypoint_idx]
        distance = np.linalg.norm(self.position[:2] - current_waypoint[:2])
        if distance < self.waypoint_threshold:
            self.current_waypoint_idx = min(self.current_waypoint_idx + 1, len(self.waypoints)-1)
            self.last_waypoint_time = time.time()

        # Store experience with GPU-ready data types
        next_state = self.get_state()
        reward = self.compute_reward(distance)
        self.episode_reward += reward
        self.replay_buffer.push(state, action, reward, next_state)

        # Train with prioritized experience replay
        if len(self.replay_buffer) > BATCH_SIZE * 10 and self.train_step % 4 == 0:
            self.train()

        # Adaptive logging
        if self.get_parameter('verbose_logging').value:
            current_time = time.time()
            if current_time - self.last_print_time >= 5:
                self.log_status(distance)
                self.last_print_time = current_time
                # Dynamic parameter adjustment
                self.noise_scale = max(0.1, 1.0 - self.episode_count * 0.002)

        self.train_step += 1
        self.noise_scale *= NOISE_DECAY
    
    def initiate_reset(self):
        """Start reset sequence with safety measures"""
        self.get_logger().warning("Initiating reset sequence")
        
        # 1. Immediately stop the drone
        self.send_zero_velocity()
        
        # 2. Freeze training activity
        self.is_resetting = True
        self.reset_start_time = time.time()
        
        # 3. Start reset process in background
        self.end_episode()
    
    def send_zero_velocity(self):
        """Emergency stop command"""
        twist = Twist()
        self.action_pub.publish(twist)
        self.get_logger().debug("Sent zero velocity command")
    
    def log_status(self, current_distance):
        altitude_error = abs(self.position[2] - TARGET_ALTITUDE)
        status_msg = [
            "\n===== System Status =====",
            f"Episode Reward: {self.episode_reward:.2f}",
            f"Episode: {self.episode_count} | Step: {self.train_step}",
            f"Successful Episodes: {self.successful_episodes}",
            f"Noise Scale: {self.noise_scale:.4f}",
            f"Current Waypoint: {self._format_waypoint()}",
            f"Position: [X: {self.position[0]:.2f}, Y: {self.position[1]:.2f}, Z: {self.position[2]:.2f}]",
            f"Altitude Error: {altitude_error:.2f}m (Target: {TARGET_ALTITUDE}m ±1m)",
            f"Action Taken: {self._format_action()}",
            f"Distance Error: {current_distance:.2f}m",
            f"Bounding Boxes: {self._format_bboxes()}",
            "=========================\n"
        ]
        self.get_logger().info('\n'.join(status_msg))

    def _format_waypoint(self):
        if self.current_waypoint_idx < len(self.waypoints):
            wp = self.waypoints[self.current_waypoint_idx]
            return f"{self.current_waypoint_idx+1}/{len(self.waypoints)} [X: {wp[0]:.2f}, Y: {wp[1]:.2f}]"
        return "No waypoint available"

    def _format_action(self):
        if self.last_action is not None:
            return f"[Vx: {self.last_action[0]:.2f}, Vz: {self.last_action[1]:.2f}, Yaw: {self.last_action[2]:.2f} rad/s]"
        return "No action recorded"

    def _format_bboxes(self):
        if not self.bboxes:
            return "No bounding boxes detected"
        return '\n'.join([f"- [{p.x:.2f}, {p.y:.2f}, {p.z:.2f}]" for p in self.bboxes[:3]]) + \
            ("\n(...)" if len(self.bboxes) > 3 else "")

    # Remaining methods (train, end_episode, save/load_checkpoint, etc.) same as previous version

def main(args=None):
    rclpy.init(args=args)
    trainer = DDPGTrainer()
    rclpy.spin(trainer)
    trainer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()