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
    """Main controller for teleoperation system - supports dual arms"""
    
    def __init__(self, workspace_limits: Optional[WorkspaceLimits] = None,
                 velocity_limiter: Optional[VelocityLimiter] = None):
        """
        Initialize teleoperation controller
        """
        self.workspace = workspace_limits or WorkspaceLimits()
        self.velocity_limiter = velocity_limiter or VelocityLimiter()
        
        # State for both hands
        self.poses = {
            "left": {
                "position": np.array([-0.2, 0.0, 0.2]), # Initial offset for left arm
                "orientation": np.array([1.0, 0.0, 0.0, 0.0])
            },
            "right": {
                "position": np.array([0.2, 0.0, 0.2]),  # Initial offset for right arm
                "orientation": np.array([1.0, 0.0, 0.0, 0.0])
            }
        }
        
        # Current state (for backward compatibility if needed, though we should use self.poses)
        self.current_position = self.poses["right"]["position"]
        self.current_orientation = self.poses["right"]["orientation"]
        
        self.last_command_time: Optional[float] = None
        
        # Statistics
        self.command_count = 0
        self.workspace_violations = 0
        self.velocity_violations = 0
        
    def process_command(self, command: DeltaCommand,
                        current_time: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray, float, Dict[str, Any]]:
        """
        Process a delta command and return target pose
        """
        if current_time is None:
            current_time = time.time()
            
        self.last_command_time = current_time
        self.command_count += 1
        
        violations = {
            'workspace_violation': False,
            'velocity_violation': False
        }
        
        # Get handedness
        handedness = command.handedness.lower()
        if handedness not in self.poses:
            handedness = "right" # Default
            
        # Get current state for this hand
        current_pos = self.poses[handedness]["position"]
        current_ori = self.poses[handedness]["orientation"]
        
        # Convert command to numpy arrays
        delta_pos = np.array([command.dx, command.dy, command.dz])
        delta_euler = np.array([command.droll, command.dpitch, command.dyaw])
        
        # Transform delta based on reference frame
        if command.reference_frame == ReferenceFrame.END_EFFECTOR:
            # Transform delta from end-effector frame to world frame
            delta_pos_world = self._transform_to_world_frame(delta_pos, current_ori)
        else:
            # Assume WORLD frame
            delta_pos_world = delta_pos
        
        # Calculate target position
        target_position = current_pos + delta_pos_world
        
        # Apply workspace limits
        if not self.workspace.contains(target_position):
            violations['workspace_violation'] = True
            self.workspace_violations += 1
            target_position = self.workspace.clamp(target_position)
        
        # Calculate target orientation
        # Convert current orientation to rotation matrix
        R_current = quaternions.quat2mat(current_ori)
        
        # Create rotation matrix from delta euler angles
        R_delta = euler.euler2mat(delta_euler[0],
                                  delta_euler[1],
                                  delta_euler[2])
        
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
        
        # Update state for this hand
        self.poses[handedness]["position"] = target_position.copy()
        self.poses[handedness]["orientation"] = target_orientation.copy()
        
        # Update legacy state for backward compatibility
        if handedness == "right":
            self.current_position = target_position.copy()
            self.current_orientation = target_orientation.copy()
        
        return target_position, target_orientation, command.gripper_state, violations
    
    def _transform_to_world_frame(self, delta_pos_ee: np.ndarray, current_orientation: np.ndarray) -> np.ndarray:
        """
        Transform delta position from end-effector frame to world frame
        """
        # Convert current orientation to rotation matrix
        R = quaternions.quat2mat(current_orientation)
        
        # Rotate delta from EE frame to world frame
        delta_pos_world = np.dot(R, delta_pos_ee)
        
        return delta_pos_world
    
    def set_current_pose(self, position: np.ndarray, orientation: np.ndarray, handedness: str = "right"):
        """
        Set the current pose (e.g., from robot feedback)
        """
        h = handedness.lower()
        if h not in self.poses:
            h = "right"
            
        self.poses[h]["position"] = position.copy()
        self.poses[h]["orientation"] = orientation.copy()
        
        # Normalize orientation
        self.poses[h]["orientation"] = self.poses[h]["orientation"] / np.linalg.norm(self.poses[h]["orientation"])
        
        if h == "right":
            self.current_position = self.poses[h]["position"].copy()
            self.current_orientation = self.poses[h]["orientation"].copy()
    
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
