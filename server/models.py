"""
Data models for teleoperation system
"""
from pydantic import BaseModel
from typing import Optional, Tuple, List
import numpy as np
from dataclasses import dataclass
from datetime import datetime


from enum import Enum

class ReferenceFrame(str, Enum):
    WORLD = "world"
    END_EFFECTOR = "end_effector"
    BASE = "base"

class DeltaCommand(BaseModel):
    """Incremental end-effector command"""
    dx: float = 0.0  # meters
    dy: float = 0.0
    dz: float = 0.0
    droll: float = 0.0  # radians
    dpitch: float = 0.0
    dyaw: float = 0.0
    reference_frame: ReferenceFrame = ReferenceFrame.END_EFFECTOR
    max_velocity: float = 0.1  # m/s
    max_angular_velocity: float = 0.5  # rad/s
    timestamp: float = 0.0
    client_id: str = "default"


class JointCommand(BaseModel):
    """Direct joint position command"""
    joints: List[float]  # Array of joint angles/values
    timestamp: float = 0.0
    client_id: str = "default"


class RobotState(BaseModel):
    """Current robot state"""
    position: Tuple[float, float, float]  # x, y, z in world frame
    orientation: Tuple[float, float, float, float]  # quaternion (w, x, y, z)
    velocity: Tuple[float, float, float]
    angular_velocity: Tuple[float, float, float]
    timestamp: float
    is_moving: bool


class TeleopStatus(BaseModel):
    """Teleoperation system status"""
    is_active: bool
    safety_gate_active: bool
    last_command_time: float
    robot_connected: bool
    backend_type: str
    workspace_violation: bool
    velocity_violation: bool


@dataclass
class WorkspaceLimits:
    """Workspace bounding box limits"""
    min_x: float = -1.0
    max_x: float = 1.0
    min_y: float = -1.0
    max_y: float = 1.0
    min_z: float = 0.0
    max_z: float = 1.5

    def contains(self, position: np.ndarray) -> bool:
        """Check if position is within workspace"""
        return (self.min_x <= position[0] <= self.max_x and
                self.min_y <= position[1] <= self.max_y and
                self.min_z <= position[2] <= self.max_z)

    def clamp(self, position: np.ndarray) -> np.ndarray:
        """Clamp position to workspace limits"""
        return np.array([
            np.clip(position[0], self.min_x, self.max_x),
            np.clip(position[1], self.min_y, self.max_y),
            np.clip(position[2], self.min_z, self.max_z)
        ])