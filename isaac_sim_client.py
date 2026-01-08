import argparse
import sys
import socket
import json
import time
import threading
import numpy as np

# 1. Parse args BEFORE SimulationApp
parser = argparse.ArgumentParser(description="Isaac Sim TCP Client")
parser.add_argument("--host", type=str, default="localhost", help="Teleop Server Host")
parser.add_argument("--port", type=int, default=9000, help="Teleop Server TCP Port")
parser.add_argument("--robot_usd", type=str, default="Franka/franka.usd", help="Path to robot USD relative to Assets root")
parser.add_argument("--ee_frame", type=str, default="panda_hand", help="End-effector frame name")
parser.add_argument("--headless", action="store_true", help="Run in headless mode")
args, unknown = parser.parse_known_args()

# 2. Start SimulationApp
from omni.isaac.kit import SimulationApp
config = {"headless": args.headless}
if args.headless:
    config["width"] = 1280
    config["height"] = 720
    config["renderer"] = "RayTracedLighting"
    # Enable WebRTC Livestreaming
    config["enable_livestream"] = True
    config["livestream_type"] = "webrtc"

simulation_app = SimulationApp(config)

# 3. Import Isaac Core (must be after SimulationApp)
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.articulations import Articulation
from omni.isaac.motion_generation import ArticulationMotionController

class IsaacSimTCPClient:
    def __init__(self, host, port, robot_usd, ee_frame):
        self.host = host
        self.port = port
        self.robot_usd = robot_usd
        self.ee_frame = ee_frame
        
        self.running = True
        self.sock = None
        
        self.world = None
        self.robot = None
        self.controller = None
        
        # State
        self.target_pos = np.array([0.5, 0.0, 0.5], dtype=np.float32)
        self.target_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32) # w, x, y, z

    def init_sim(self):
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        
        assets_root = get_assets_root_path()
        if not assets_root:
            print("Warning: Could not find Nucleus assets root. Using local if available or failing.")
            robot_url = self.robot_usd
        else:
            robot_url = f"{assets_root}/Isaac/Robots/{self.robot_usd}"
            
        print(f"Loading robot from: {robot_url}")
        
        # Add robot to scene
        self.world.scene.add(Articulation(prim_path="/World/Robot", usd_path=robot_url, name="robot"))
        self.world.reset()
        
        self.robot = self.world.scene.get_object("robot")
        self.controller = ArticulationMotionController(self.robot)
        
        # Initial warm-up
        for _ in range(10):
            self.world.step(render=False)

    def network_loop(self):
        while self.running:
            # Try to connect
            if self.sock is None:
                try:
                    print(f"Connecting to {self.host}:{self.port}...")
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.connect((self.host, self.port))
                    self.sock = s
                    print("Connected to server!")
                except Exception as e:
                    print(f"Connection failed: {e}. Retrying in 3s...")
                    time.sleep(3)
                    continue

            # Receive data
            try:
                data = self.sock.recv(4096)
                if not data:
                    print("Server disconnected.")
                    self.sock.close()
                    self.sock = None
                    continue
                
                # Process lines
                buffer = data.decode("utf-8")
                for line in buffer.split("\n"):
                    if not line.strip(): continue
                    try:
                        msg = json.loads(line)
                        if msg.get("type") == "command":
                            payload = msg.get("payload", {})
                            if "target_position" in payload:
                                self.target_pos = np.array(payload["target_position"], dtype=np.float32)
                            if "target_orientation" in payload:
                                self.target_quat = np.array(payload["target_orientation"], dtype=np.float32)
                    except json.JSONDecodeError:
                        pass
            except Exception as e:
                print(f"Network error: {e}")
                if self.sock:
                    self.sock.close()
                self.sock = None

    def run(self):
        self.init_sim()
        
        # Start network thread
        net_thread = threading.Thread(target=self.network_loop, daemon=True)
        net_thread.start()
        
        print("Starting simulation loop...")
        
        # Enable camera for livestream
        if self.world:
            self.world.reset()

        while simulation_app.is_running():
            self.world.step(render=True)
            if self.running and self.controller:
                # Apply IK
                action = self.controller.compute_inverse_kinematics(
                    target_end_effector_position=self.target_pos,
                    target_end_effector_orientation=self.target_quat,
                    end_effector_frame_name=self.ee_frame
                )
                self.robot.apply_action(action)
        
        self.running = False
        simulation_app.close()

if __name__ == "__main__":
    client = IsaacSimTCPClient(args.host, args.port, args.robot_usd, args.ee_frame)
    client.run()
