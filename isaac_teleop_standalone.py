# isaac_teleop_standalone.py
"""
Universal Teleoperation Platform - Isaac Sim Standalone Version
Run this script with Isaac Sim's python.sh:
~/.local/share/ov/pkg/isaac_sim-2023.1.0/python.sh isaac_teleop_standalone.py configs/robots/your_robot.yaml
"""

import argparse
import sys
import os
import yaml
import numpy as np
import threading
import time
from typing import Dict, Any, Optional

# Isaac Sim Core Imports (will only work in Isaac Sim environment)
try:
    from omni.isaac.kit import SimulationApp
    
    # Configuration for SimulationApp
    CONFIG = {
        "width": 1920,
        "height": 1080,
        "window_width": 1920,
        "window_height": 1080,
        "headless": False,
        "renderer": "RayTracedLighting",
        "display_options": 3286,  # Show Grid + Axis
    }
    
    # Start SimulationApp first!
    simulation_app = SimulationApp(CONFIG)
    
    from omni.isaac.core import World
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.articulations import Articulation
    from omni.isaac.motion_generation import ArticulationMotionController, LulaKinematicsSolver
    from omni.isaac.core.utils.nucleus import get_assets_root_path
    import carb
    
except ImportError:
    print("❌ Error: This script must be run within Isaac Sim's Python environment.")
    sys.exit(1)

# API Server Imports
import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

class TeleopCommand(BaseModel):
    """Standardized teleop command"""
    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0
    droll: float = 0.0
    dpitch: float = 0.0
    dyaw: float = 0.0
    gripper: float = 0.0
    deadman: bool = False

class UniversalTeleopSystem:
    def __init__(self, config_path: str):
        self.config_path = config_path
        self.config = self._load_config(config_path)
        
        self.world: Optional[World] = None
        self.robot: Optional[Articulation] = None
        self.controller: Optional[ArticulationMotionController] = None
        
        # Teleop State
        self.running = True
        self.latest_command: Optional[Dict] = None
        self.command_lock = threading.Lock()
        
        # Robot State
        self.target_pos = None
        self.target_rot = None  # Quaternion
        
        # Initialize
        self._init_simulation()
        self._start_api_server()

    def _load_config(self, path: str) -> Dict:
        if not os.path.exists(path):
            raise FileNotFoundError(f"Config file not found: {path}")
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    def _init_simulation(self):
        print(f"🌍 Initializing Isaac Sim World...")
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        
        # Add Lighting
        from pxr import UsdLux
        stage = self.world.stage
        light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
        light.CreateIntensityAttr(1000)

        # Load Robot from Config
        robot_cfg = self.config['isaac_sim']
        usd_path = robot_cfg['usd_path']
        prim_path = robot_cfg.get('prim_path', f"/World/{self.config['robot_name']}")
        
        # Resolve Nucleus path if needed
        if usd_path.startswith("omniverse://"):
            # If strictly local, you might need to handle this differently
            pass
        elif not os.path.isabs(usd_path):
            # Convert relative path to absolute based on CWD
            usd_path = os.path.abspath(usd_path)
        
        print(f"🤖 Loading Robot: {self.config['robot_name']} from {usd_path}")
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
        
        # Initialize Robot Articulation
        self.robot = Articulation(prim_path=prim_path, name=self.config['robot_name'])
        self.world.scene.add(self.robot)
        
        # Initialize Controller (if Kinematics config exists)
        # Note: For custom robots like RM65, we might need to setup Lula manually
        # For now, we'll use ArticulationMotionController as a base
        self.controller = ArticulationMotionController(self.robot)
        
        self.world.reset()
        
        # Set initial pose
        if 'initial_pose' in self.config:
            # TODO: Set initial joint positions
            pass
            
        print("✅ Simulation Initialized")

    def _start_api_server(self):
        """Start FastAPI server in a separate thread"""
        app = FastAPI(title="Isaac Teleop Server")

        @app.post("/command")
        async def receive_command(cmd: TeleopCommand):
            with self.command_lock:
                self.latest_command = cmd.dict()
            return {"status": "ok"}

        @app.get("/state")
        async def get_state():
            if not self.robot:
                return {"status": "not_ready"}
            
            try:
                pos, rot = self.robot.get_world_pose()
                return {
                    "robot": self.config['robot_name'],
                    "position": pos.tolist(),
                    "orientation": rot.tolist(),  # w, x, y, z
                    "joints": self.robot.get_joint_positions().tolist()
                }
            except Exception as e:
                return {"status": "error", "message": str(e)}

        def run_server():
            print(f"🚀 API Server starting on port 8000...")
            uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")

        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()

    def process_teleop_command(self):
        """Apply latest teleop command to robot"""
        cmd = None
        with self.command_lock:
            if self.latest_command:
                cmd = self.latest_command
                self.latest_command = None  # Consume command
        
        if not cmd or not self.robot:
            return

        # Simple Cartesian Control Logic
        # 1. Get current EE pose
        ee_frame = self.config.get('kinematics', {}).get('end_effector_frame', 'tool0')
        
        # Note: Getting explicit frame pose requires UsdGeom or Articulation View
        # For simplicity in this proto, we might control the robot base or assume specific chain
        # Ideally, use self.robot.end_effector... if defined
        
        # For now, let's just log that we received it
        # In real implementation, we feed this to RmpFlow or IK
        pass

    def run(self):
        while simulation_app.is_running():
            self.world.step(render=True)
            if self.world.is_playing():
                self.process_teleop_command()
            
        simulation_app.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("config", help="Path to robot YAML config")
    args = parser.parse_args()
    
    teleop = UniversalTeleopSystem(args.config)
    teleop.run()
