import os
import yaml
import glob
from typing import Dict, Any, List, Optional
from pydantic import BaseModel

class RobotConfig(BaseModel):
    robot_name: str
    dof: int
    isaac_sim: Optional[Dict[str, Any]] = None
    real_robot: Optional[Dict[str, Any]] = None
    workspace_limits: Optional[Dict[str, Any]] = None
    controller: Optional[Dict[str, Any]] = None
    manufacturer: str = "Unknown"
    file_path: Optional[str] = None # Added field to track source file

    def has_simulation_backend(self) -> bool:
        return self.isaac_sim is not None

    def has_real_backend(self) -> bool:
        return self.real_robot is not None

    def validate(self):
        if not self.robot_name:
            raise ValueError("Robot name is required")
        if self.dof <= 0:
            raise ValueError("DOF must be positive")
        if not self.has_simulation_backend() and not self.has_real_backend():
            raise ValueError("At least one backend (simulation or real) must be configured")

class RobotRegistry:
    def __init__(self, config_dir: str = None):
        if config_dir is None:
            # Default to configs/robots relative to repo root
            repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            config_dir = os.path.join(repo_root, "configs", "robots")
        
        self.config_dir = config_dir
        self.robots: Dict[str, RobotConfig] = {}
        self._load_all_configs()

    def _load_all_configs(self):
        if not os.path.exists(self.config_dir):
            return

        yaml_files = glob.glob(os.path.join(self.config_dir, "*.yaml"))
        for yaml_file in yaml_files:
            try:
                with open(yaml_file, 'r') as f:
                    data = yaml.safe_load(f)
                    if data and 'robot_name' in data:
                        config = RobotConfig(**data)
                        config.file_path = yaml_file
                        self.robots[config.robot_name] = config
                        # Also register by lowercase name for easier lookup
                        self.robots[config.robot_name.lower()] = config
            except Exception as e:
                print(f"Error loading config {yaml_file}: {e}")

    def list_robots(self) -> List[str]:
        # Return unique robot names (preserving case from config)
        return list(set([config.robot_name for config in self.robots.values()]))

    def list_robots_with_sim(self) -> List[str]:
        return list(set([config.robot_name for config in self.robots.values() if config.has_simulation_backend()]))

    def get_robot(self, name: str) -> Optional[RobotConfig]:
        return self.robots.get(name) or self.robots.get(name.lower())

    def get_summary(self) -> Dict[str, Any]:
        return {
            "total_robots": len(self.list_robots()),
            "robots": self.list_robots()
        }

def load_robot_config(config_path: str) -> RobotConfig:
    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)
    return RobotConfig(**data)
