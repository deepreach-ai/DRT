import os
import time
import numpy as np
from typing import Tuple, Optional, Dict, Any, Union
from enum import Enum
from robot_backend import RobotBackend, BackendStatus, BackendFactory
from robot_config import load_robot_config, RobotConfig

class RobotMode(Enum):
    SIMULATION = "simulation"
    REAL = "real"

class UnifiedRobotBackend(RobotBackend):
    def __init__(self, config_path: str, mode: Union[str, RobotMode] = "simulation"):
        self.config = load_robot_config(config_path)
        super().__init__(name=self.config.robot_name)
        
        self.mode = RobotMode(mode) if isinstance(mode, str) else mode
        self._backend: Optional[RobotBackend] = None
        self._create_backend()

    def _create_backend(self):
        if self.mode == RobotMode.SIMULATION:
            if not self.config.has_simulation_backend():
                raise ValueError(f"Robot {self.config.robot_name} has no simulation backend configured")
            
            # Create Isaac Sim backend
            # Map config to IsaacSimBackend args
            isaac_config = self.config.isaac_sim
            
            # Filter args for IsaacSimBackend (only accepts host, port, name)
            backend_args = {
                "name": self.config.robot_name
            }
            if "host" in isaac_config:
                backend_args["host"] = isaac_config["host"]
            if "port" in isaac_config:
                backend_args["port"] = isaac_config["port"]
                
            self._backend = BackendFactory.create_backend("isaac", **backend_args)
            
        elif self.mode == RobotMode.REAL:
            if not self.config.has_real_backend():
                raise ValueError(f"Robot {self.config.robot_name} has no real robot backend configured")
            
            real_config = self.config.real_robot
            backend_type = real_config.get("backend_type", "mock") # Default to mock if not specified for safety
            
            # If backend_type is 'robot_sdk' or similar, we might need a specific handler.
            # For this implementation, we'll try to use the factory.
            # If the factory doesn't support the type, we might default to Mock for demo.
            try:
                self._backend = BackendFactory.create_backend(backend_type, **real_config.get("connection", {}))
            except ValueError:
                # Fallback to mock if type not supported (e.g. 'realman_sdk')
                print(f"Warning: Backend type '{backend_type}' not found in factory. Using MockBackend.")
                self._backend = BackendFactory.create_backend("mock", name=f"{self.config.robot_name}_real")

    def switch_mode(self, mode: Union[str, RobotMode]):
        new_mode = RobotMode(mode) if isinstance(mode, str) else mode
        if new_mode == self.mode:
            return

        print(f"🔄 Switching from {self.mode.value} to {new_mode.value}...")
        
        # Disconnect current backend
        if self._backend and self._backend.is_connected():
            print(f"🔌 Disconnecting from {self.mode.value} robot...")
            self.disconnect()
            print(f"✅ Disconnected from {self.mode.value} robot")

        self.mode = new_mode
        self._create_backend()
        
        # Connect new backend
        print(f"🔌 Connecting to {self.mode.value} robot...")
        if self.connect():
            print(f"✅ Connected to {self.mode.value} robot")
            print(f"✅ Successfully switched to {self.mode.value} mode!")
            print(f"   Now controlling: {self.config.robot_name} ({self.mode.value})")
        else:
            print(f"❌ Failed to connect to {self.mode.value} robot")

    def connect(self) -> bool:
        if self._backend:
            success = self._backend.connect()
            if success:
                self.status = BackendStatus.CONNECTED
            return success
        return False

    def disconnect(self):
        if self._backend:
            self._backend.disconnect()
        self.status = BackendStatus.DISCONNECTED

    def send_target_pose(self, position: np.ndarray, orientation: np.ndarray, velocity_limit: float = 0.1, gripper_state: float = -1.0) -> bool:
        if self._backend:
            return self._backend.send_target_pose(position, orientation, velocity_limit, gripper_state)
        return False

    def get_current_pose(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if self._backend:
            return self._backend.get_current_pose()
        return None, None

    def get_status(self) -> Dict[str, Any]:
        status = {
            "mode": self.mode.value,
            "robot_name": self.config.robot_name
        }
        if self._backend:
            status.update(self._backend.get_status())
        return status
    
    # Extra methods required by test_new_features.py
    def get_workspace_limits(self) -> Dict[str, Any]:
        return self.config.workspace_limits or {
            "x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 1.0]
        }

    def get_max_velocity(self) -> float:
        if self.config.controller:
            return self.config.controller.get("max_velocity", 0.5)
        return 0.5
