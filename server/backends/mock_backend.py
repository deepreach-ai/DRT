"""
Mock backend for testing without actual robot/simulation
"""
from typing import Tuple, Optional, Dict, Any
import numpy as np
import time
import threading

from robot_backend import RobotBackend, BackendStatus


class MockRobotBackend(RobotBackend):
    """Mock backend for testing without actual robot/simulation"""
    
    def __init__(self, name: str = "mock_robot"):
        super().__init__(name)
        self.current_position = np.array([0.0, 0.0, 0.5])
        self.current_orientation = np.array([1.0, 0.0, 0.0, 0.0])
        self.command_count = 0
        self.simulated_latency = 0.01  # seconds
        
    def connect(self) -> bool:
        """Simulate connection"""
        print(f"[MockBackend] Connecting to {self.name}...")
        time.sleep(0.1)  # Simulate connection delay
        self.status = BackendStatus.CONNECTED
        self.last_update_time = time.time()
        print(f"[MockBackend] Connected to {self.name}")
        return True
    
    def disconnect(self):
        """Simulate disconnection"""
        print(f"[MockBackend] Disconnecting from {self.name}...")
        self.status = BackendStatus.DISCONNECTED
        print(f"[MockBackend] Disconnected from {self.name}")
    
    def send_target_pose(self, position: np.ndarray, 
                         orientation: np.ndarray,
                         velocity_limit: float = 0.1) -> bool:
        """Simulate sending target pose"""
        if not self.is_connected():
            return False
            
        # Simulate network latency
        time.sleep(self.simulated_latency)
        
        with self.update_lock:
            # Simple interpolation for simulation
            alpha = min(velocity_limit * 0.1, 1.0)  # Simple interpolation factor
            self.current_position = self.current_position * (1 - alpha) + position * alpha
            self.current_orientation = orientation  # Direct set for simplicity
            self.command_count += 1
            self.last_update_time = time.time()
            
        print(f"[MockBackend] 📥 Received EE Command | Pos: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] | Rot: [{orientation[0]:.3f}, {orientation[1]:.3f}, {orientation[2]:.3f}, {orientation[3]:.3f}]")
        return True
    
    def get_current_pose(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get simulated current pose"""
        if not self.is_connected():
            return None, None
            
        with self.update_lock:
            return self.current_position.copy(), self.current_orientation.copy()

    def get_joint_positions(self) -> Optional[np.ndarray]:
        """Get simulated joint positions"""
        if not self.is_connected():
            return None
        # Return 6 zeros for mock 6-DoF arm
        return np.zeros(6)
    
    def get_status(self) -> Dict[str, Any]:
        """Get mock backend status"""
        return {
            'name': self.name,
            'status': self.status.value,
            'command_count': self.command_count,
            'current_position': self.current_position.tolist(),
            'current_orientation': self.current_orientation.tolist(),
            'simulated_latency': self.simulated_latency,
            'last_update': self.last_update_time
        }
