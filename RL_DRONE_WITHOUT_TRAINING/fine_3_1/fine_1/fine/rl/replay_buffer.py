import random
import numpy as np
from collections import deque

class ReplayBuffer:
    def __init__(self, capacity):
        """
        Initialize the ReplayBuffer with a fixed maximum capacity.
        
        Args:
            capacity (int): Maximum number of experiences to store.
        """
        self.buffer = deque(maxlen=capacity)

    def push(self, state, action, reward, next_state):
        """
        Stores a new experience in the buffer.
        
        Args:
            state (np.array): The current state, e.g., a 17-dim vector (2 velocity components + 5 bounding boxes * 3).
            action (np.array): The action taken (e.g., a 3-dim vector: vx, vz, yaw).
            reward (float): The reward received.
            next_state (np.array): The next state after the action.
        """
        experience = (state, action, reward, next_state)
        self.buffer.append(experience)

    def sample(self, batch_size):
        """
        Randomly sample a batch of experiences from the buffer.
        
        Args:
            batch_size (int): Number of experiences to sample.
            
        Returns:
            tuple: A tuple containing batch arrays for states, actions, rewards, and next_states.
                   Each element is a NumPy array with an appropriate dtype.
        """
        batch = random.sample(self.buffer, batch_size)
        states, actions, rewards, next_states = zip(*batch)
        return (
            np.array(states, dtype=np.float32),
            np.array(actions, dtype=np.float32),
            np.array(rewards, dtype=np.float32),
            np.array(next_states, dtype=np.float32)
        )

    def __len__(self):
        """
        Returns the current number of experiences stored in the buffer.
        """
        return len(self.buffer)
