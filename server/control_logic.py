"""
Control logic for teleoperation including workspace limits and velocity limiting
"""
import numpy as np
from typing import Tuple, Optional, Dict, Any
import time
from transforms3d import quaternions, euler

from models import DeltaCommand, WorkspaceLimits, ReferenceFrame
from safety_gate import VelocityLimiter


class TeleoperationController:
    """Main controller for teleoperation system"""
    
    def __init__(self, workspace_limits: Optional[WorkspaceLimits] = None,
                 velocity_limiter: Optional[VelocityLimiter] = None):
        """
        Initialize teleoperation controller
        
        Args:
            workspace_limits: Workspace bounding box
            velocity_limiter: Velocity limiting module
        """
        self.workspace = workspace_limits or WorkspaceLimits()
        self.velocity_limiter = velocity_limiter or VelocityLimiter()
        
        # Current state
        self.current_position = np.array([0.0, 0.0, 0.0])  # World frame
        self.current_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion (w, x, y, z)
        self.last_command_time: Optional[float] = None
        
        # Statistics
        self.command_count = 0
        self.workspace_violations = 0
        self.velocity_violations = 0
        
    def process_command(self, command: DeltaCommand,
                        current_time: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray, float, Dict[str, Any]]:
        """
        Process a delta command and return target pose
        
        Args:
            command: Delta command from client
            current_time: Current timestamp (uses time.time() if None)
            
        Returns:
            target_position: Target position in world frame [x, y, z]
            target_orientation: Target orientation as quaternion [w, x, y, z]
            gripper_state: Gripper state (0.0-1.0)
            violations: Dictionary of any violations that occurred
        """
        if current_time is None:
            current_time = time.time()
            
        # Calculate time delta
        dt = 0.1  # Default if first command
        if self.last_command_time is not None:
            dt = current_time - self.last_command_time
            dt = max(dt, 0.001)  # Prevent division by zero
            dt = min(dt, 1.0)    # Cap at 1 second for stability
            
        self.last_command_time = current_time
        self.command_count += 1
        
        violations = {
            'workspace_violation': False,
            'velocity_violation': False
        }
        
        # Convert command to numpy arrays
        delta_pos = np.array([command.dx, command.dy, command.dz])
        delta_euler = np.array([command.droll, command.dpitch, command.dyaw])
        
        # Apply velocity limits
        self.velocity_limiter.set_limits(command.max_velocity, command.max_angular_velocity)
        delta_pos_limited, delta_euler_limited = self.velocity_limiter.limit(
            delta_pos, delta_euler, dt
        )
        
        # Check if velocity limiting was applied
        if not np.allclose(delta_pos, delta_pos_limited) or not np.allclose(delta_euler, delta_euler_limited):
            violations['velocity_violation'] = True
            self.velocity_violations += 1
        
        # Transform delta based on reference frame
        if command.reference_frame == ReferenceFrame.END_EFFECTOR:
            # Transform delta from end-effector frame to world frame
            delta_pos_world = self._transform_to_world_frame(delta_pos_limited)
        else:
            # Assume WORLD frame (BASE is treated as WORLD for simplicity in this fixed-base setup)
            delta_pos_world = delta_pos_limited
        
        # Calculate target position
        target_position = self.current_position + delta_pos_world
        
        # Apply workspace limits
        if not self.workspace.contains(target_position):
            violations['workspace_violation'] = True
            self.workspace_violations += 1
            target_position = self.workspace.clamp(target_position)
        
        # Calculate target orientation
        # Convert current orientation to rotation matrix
        R_current = quaternions.quat2mat(self.current_orientation)
        
        # Create rotation matrix from delta euler angles
        R_delta = euler.euler2mat(delta_euler_limited[0],
                                  delta_euler_limited[1],
                                  delta_euler_limited[2])
        
        # Combine rotations based on reference frame
        if command.reference_frame == ReferenceFrame.END_EFFECTOR:
            # Local rotation: R_new = R_current * R_delta
            R_target = np.dot(R_current, R_delta)
        else:
            # Global rotation: R_new = R_delta * R_current
            R_target = np.dot(R_delta, R_current)
        
        # Convert back to quaternion
        target_orientation = quaternions.mat2quat(R_target)
        
        # Normalize quaternion
        target_orientation = target_orientation / np.linalg.norm(target_orientation)
        
        # Update current state
        self.current_position = target_position.copy()
        self.current_orientation = target_orientation.copy()
        
        return target_position, target_orientation, command.gripper_state, violations
    
    def _transform_to_world_frame(self, delta_pos_ee: np.ndarray) -> np.ndarray:
        """
        Transform delta position from end-effector frame to world frame
        
        Args:
            delta_pos_ee: Delta position in end-effector frame
            
        Returns:
            Delta position in world frame
        """
        # Convert current orientation to rotation matrix
        R = quaternions.quat2mat(self.current_orientation)
        
        # Rotate delta from EE frame to world frame
        delta_pos_world = np.dot(R, delta_pos_ee)
        
        return delta_pos_world
    
    def set_current_pose(self, position: np.ndarray, orientation: np.ndarray):
        """
        Set the current pose (e.g., from robot feedback)
        
        Args:
            position: Current position in world frame [x, y, z]
            orientation: Current orientation as quaternion [w, x, y, z]
        """
        self.current_position = position.copy()
        self.current_orientation = orientation.copy()
        
        # Normalize orientation
        self.current_orientation = self.current_orientation / np.linalg.norm(self.current_orientation)
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get controller statistics"""
        return {
            'command_count': self.command_count,
            'workspace_violations': self.workspace_violations,
            'velocity_violations': self.velocity_violations,
            'current_position': self.current_position.tolist(),
            'current_orientation': self.current_orientation.tolist()
        }
    
    def reset(self):
        """Reset controller state"""
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_orientation = np.array([1.0, 0.0, 0.0, 0.0])
        self.last_command_time = None
        self.command_count = 0
        self.workspace_violations = 0
        self.velocity_violations = 0
