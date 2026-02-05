"""
Abstract base class for robot backends
"""
from abc import ABC, abstractmethod
from typing import Tuple, Optional, Dict, Any
import numpy as np
import threading
import time
from enum import Enum


class BackendStatus(Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"


class RobotBackend(ABC):
    """Abstract base class for robot backends"""
    
    def __init__(self, name: str = "robot"):
        self.name = name
        self.status = BackendStatus.DISCONNECTED
        self.last_update_time = 0.0
        self.update_lock = threading.Lock()
        
    @abstractmethod
    def connect(self) -> bool:
        """Connect to the robot/simulation"""
        pass
    
    @abstractmethod
    def disconnect(self):
        """Disconnect from the robot/simulation"""
        pass
    
    @abstractmethod
    def send_target_pose(self, position: np.ndarray, 
                         orientation: np.ndarray,
                         velocity_limit: float = 0.1) -> bool:
        """
        Send target pose to robot
        
        Args:
            position: Target position [x, y, z] in meters
            orientation: Target orientation as quaternion [w, x, y, z]
            velocity_limit: Maximum velocity (m/s)
            
        Returns:
            True if command was sent successfully
        """
        pass
    
    @abstractmethod
    def get_current_pose(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get current robot pose
        
        Returns:
            position: Current position [x, y, z] or None if not available
            orientation: Current orientation as quaternion [w, x, y, z] or None
        """
        pass
    
    @abstractmethod
    def get_status(self) -> Dict[str, Any]:
        """Get backend-specific status information"""
        pass
    
    def is_connected(self) -> bool:
        """Check if backend is connected"""
        return self.status == BackendStatus.CONNECTED


class BackendFactory:
    """Factory for creating robot backends"""
    
    @staticmethod
    def create_backend(backend_type: str, **kwargs) -> RobotBackend:
        """
        Create a robot backend based on type
        
        Args:
            backend_type: Type of backend ('mock', 'isaac', 'mujoco')
            **kwargs: Backend-specific parameters
            
        Returns:
            RobotBackend instance
        """
        if backend_type.lower() == 'mock':
            from backends.mock_backend import MockRobotBackend
            name = kwargs.get('name', 'mock_robot')
            return MockRobotBackend(name=name)
            
        elif backend_type.lower() == 'isaac':
            from backends.isaac_backend import IsaacSimBackend
            return IsaacSimBackend(**kwargs)

        elif backend_type.lower() == 'mujoco':
            from backends.mujoco_backend import MujocoRobotBackend
            name = kwargs.pop('name', 'mujoco_robot')
            return MujocoRobotBackend(name=name, **kwargs)
            
        elif backend_type.lower() == 'soarm' or backend_type.lower() == 'so101':
            from backends.soarm_backend import SOARMBackend
            port = kwargs.get('port', '/dev/tty.usbmodem5B3E1187881')
            name = kwargs.get('name', 'so_arm')
            return SOARMBackend(port=port, name=name)
        
        elif backend_type.lower() == 'mock_vr' or backend_type.lower() == 'mockvr':
            from backends.mock_vr_backend import MockVRBackend
            name = kwargs.get('name', 'mock_vr')
            return MockVRBackend(name=name)
            
        else:
            raise ValueError(f"Unknown backend type: {backend_type}")
