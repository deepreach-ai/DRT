"""
Safety mechanisms for teleoperation
"""
import time
from typing import Optional, Callable
import numpy as np
from typing import Tuple



class SafetyGate:
    """Deadman switch safety mechanism"""
    
    def __init__(self, timeout: float = 0.5, activation_threshold: float = 0.001):
        """
        Args:
            timeout: seconds without heartbeat before deactivation
            activation_threshold: minimum command magnitude to keep active
        """
        self.timeout = timeout
        self.activation_threshold = activation_threshold
        self.last_heartbeat: Optional[float] = None
        self.last_command_magnitude: float = 0.0
        self.is_enabled: bool = True
        
    def update(self, command: np.ndarray, timestamp: float) -> bool:
        """
        Update safety gate with new command
        
        Args:
            command: 6D command vector [dx, dy, dz, droll, dpitch, dyaw]
            timestamp: current time
            
        Returns:
            True if safety gate is active
        """
        linear_magnitude = np.linalg.norm(command[:3])
        angular_magnitude = np.linalg.norm(command[3:])
        command_magnitude = max(linear_magnitude, angular_magnitude)
        
        # Consider both linear and angular deltas, and allow equality
        if command_magnitude >= self.activation_threshold:
            self.last_heartbeat = timestamp
            self.last_command_magnitude = command_magnitude
            return True
        elif self.last_heartbeat is not None and (timestamp - self.last_heartbeat) < self.timeout:
            # Still within timeout window
            return True
        else:
            # No recent activity, deactivate
            self.last_heartbeat = None
            return False
    
    def is_active(self, current_time: Optional[float] = None) -> bool:
        """Check if safety gate is currently active"""
        if not self.is_enabled:
            return True
            
        if self.last_heartbeat is None:
            return False
            
        if current_time is None:
            current_time = time.time()
            
        return (current_time - self.last_heartbeat) < self.timeout
    
    def force_activate(self):
        """Force activate the safety gate (for testing)"""
        self.last_heartbeat = time.time()
        
    def reset(self):
        """Reset safety gate to inactive state"""
        self.last_heartbeat = None
        self.last_command_magnitude = 0.0


class VelocityLimiter:
    """Apply velocity limits to commands"""
    
    def __init__(self, max_linear_velocity: float = 0.5,
                 max_angular_velocity: float = 1.0):
        self.max_linear = max_linear_velocity
        self.max_angular = max_angular_velocity
        
    def set_limits(self, max_linear_velocity: float, max_angular_velocity: float):
        self.max_linear = max_linear_velocity
        self.max_angular = max_angular_velocity
        
    def limit(self, delta_pos: np.ndarray, delta_rot: np.ndarray,
              dt: float = 0.1) -> Tuple[np.ndarray, np.ndarray]:
        """
        Limit velocity to safe ranges
        
        Args:
            delta_pos: [dx, dy, dz]
            delta_rot: [droll, dpitch, dyaw]
            dt: time step in seconds
            
        Returns:
            Limited delta_pos, delta_rot
        """
        # Calculate current velocities
        linear_velocity = np.linalg.norm(delta_pos) / dt if dt > 0 else 0
        angular_velocity = np.linalg.norm(delta_rot) / dt if dt > 0 else 0
        
        # Apply limits
        if linear_velocity > self.max_linear:
            scale = self.max_linear / linear_velocity
            delta_pos = delta_pos * scale
            
        if angular_velocity > self.max_angular:
            scale = self.max_angular / angular_velocity
            delta_rot = delta_rot * scale
            
        return delta_pos, delta_rot
