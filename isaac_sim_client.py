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
parser.add_argument("--enable-livestream", action="store_true", default=True, help="Enable WebRTC livestream")
args, unknown = parser.parse_known_args()

# 2. Start SimulationApp with proper livestream config
# WARNING: Do NOT import omni.* or isaacsim.* before SimulationApp is created!
# Only stdlib imports are allowed before this block.

from isaacsim import SimulationApp
import os

config = {
    "headless": args.headless,
    "width": 1280,
    "height": 720,
}

# Only configure livestream if explicitly enabled and in headless mode
if args.headless and args.enable_livestream:
    config.update({
        "renderer": "RayTracedLighting",
        # Fix: Isaac Sim 4.0+ uses a different kit file or doesn't need explicit experience
        # If running from python.sh, it handles the kit loading.
        # But if we must specify, try to detect or omit if problematic.
        # For now, let's omit 'experience' as it's causing "File doesn't exist" errors
        # and SimulationApp usually picks a default.
        # "experience": f'{os.environ.get("EXP_PATH", "")}/omni.isaac.sim.python.kit',
    })

simulation_app = SimulationApp(config)

import carb
import omni.ext

# Configure livestream AFTER SimulationApp is created
if args.headless and args.enable_livestream:
    print("Configuring WebRTC Livestream...")
    
    settings = carb.settings.get_settings()
    
    # Enable required extensions
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    
    # Core streaming extensions - Using Native Streaming for better Cloud stability
    extensions_to_enable = [
        "omni.kit.livestream.native",
        "omni.kit.livestream.app",
    ]
    
    for ext_id in extensions_to_enable:
        try:
            if not ext_manager.is_extension_enabled(ext_id):
                ext_manager.set_extension_enabled_immediate(ext_id, True)
                print(f"✅ Enabled extension: {ext_id}")
        except Exception as e:
            print(f"⚠️  Could not enable {ext_id}: {e}")
    
    # Wait for extensions to initialize
    time.sleep(2)
    
    # Configure Native Livestream settings
    # See: https://docs.omniverse.nvidia.com/isaacsim/latest/manual_standalone_python.html#livestreaming
    settings.set_string("/exts/omni.kit.livestream.app/primaryStream/streamType", "native")
    settings.set_int("/exts/omni.kit.livestream.app/primaryStream/signalPort", 49100)
    settings.set_int("/exts/omni.kit.livestream.app/primaryStream/streamPort", 47998)
    settings.set_bool("/exts/omni.kit.livestream.app/primaryStream/allowDynamicResize", False)
    
    # Get public IP for EC2 (useful for logging, though client connects to IP directly)
    public_ip = None
    try:
        import urllib.request
        with urllib.request.urlopen("http://169.254.169.254/latest/meta-data/public-ipv4", timeout=1.0) as resp:
            public_ip = resp.read().decode("utf-8").strip()
            print(f"📡 Detected public IP: {public_ip}")
    except Exception:
        print("⚠️  Could not detect public IP (not on EC2?)")
    
    print("✅ Native Livestream configured")
    print(f"PLEASE USE OMNIVERSE STREAMING CLIENT to connect to: {public_ip or 'localhost'}")

# 3. Import Isaac Core (must be after SimulationApp)
# Isaac Sim 4.0+ Migration: Many modules moved to 'isaacsim' namespace
# but we need to check if we are on 4.0 or older.
# The error "ImportError: cannot import name 'ArticulationMotionController' from 'omni.isaac.motion_generation'"
# suggests we are on a version where this class was moved or renamed.

try:
    # Try new location (Isaac Sim 4.0+)
    from omni.isaac.core import World
    from omni.isaac.core.utils.nucleus import get_assets_root_path
    from omni.isaac.core.articulations import Articulation
    
    # In Isaac Sim 4.0, ArticulationMotionController might be in a different place or deprecated.
    # It was part of omni.isaac.motion_generation.
    # If it fails, we might need to use a different controller or check where it went.
    # Based on docs, LulaKinematicsSolver is the replacement for IK.
    
    try:
        from omni.isaac.motion_generation import ArticulationMotionController
    except ImportError:
        # Fallback or alternative for 4.0
        # If ArticulationMotionController is gone, we might need to use LulaKinematicsSolver directly
        # For now, let's try to find it in the new namespace if possible
        # Or just use simple Articulation control without the MotionController wrapper if we can't find it.
        print("⚠️ ArticulationMotionController not found in omni.isaac.motion_generation. Checking alternatives...")
        try:
            from omni.isaac.motion_generation.articulation_motion_controller import ArticulationMotionController
        except ImportError:
            print("❌ Could not import ArticulationMotionController. IK will be disabled.")
            ArticulationMotionController = None

except ImportError:
    # Fallback to old imports if needed (unlikely given the error log)
    pass

import os

import requests
import io
try:
    from PIL import Image
except ImportError:
    import pip
    pip.main(['install', 'pillow'])
    from PIL import Image

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
        self.gripper_state = 0.0 # 0.0 (open) to 1.0 (closed)

    def init_sim(self):
        print("Initializing simulation world...")
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        
        # Add lighting
        from pxr import UsdLux, UsdGeom
        stage = self.world.stage
        light_prim = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
        light_prim.CreateIntensityAttr(1000)

        # Add camera for livestream
        from omni.isaac.core.prims import XFormPrim
        camera_path = "/World/Camera"
        camera = UsdGeom.Camera.Define(stage, camera_path)
        camera.CreateFocalLengthAttr(24)
        
        cam_prim = XFormPrim(camera_path)
        cam_prim.set_world_pose(
            position=np.array([2.0, 2.0, 1.5]), 
            orientation=np.array([0.85, 0.15, 0.35, 0.35])  # Looking at origin
        )
        
        # Setup Render Product for capture
        import omni.replicator.core as rep
        self.render_product = rep.create.render_product(camera_path, (960, 540))
        self.rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        self.rgb_annotator.attach([self.render_product])
        
        # Load robot
        assets_root = get_assets_root_path()
        
        # Check if local path exists
        if os.path.exists(self.robot_usd):
            robot_url = os.path.abspath(self.robot_usd)
            print(f"Loading local robot USD: {robot_url}")
        elif assets_root:
            robot_url = f"{assets_root}/Isaac/Robots/{self.robot_usd}"
            print(f"Loading Nucleus robot USD: {robot_url}")
        else:
            print("⚠️  Warning: Could not find Nucleus assets root and file not found locally")
            robot_url = self.robot_usd
            
        print(f"Final robot URL: {robot_url}")
        
        # In Isaac Sim 4.0, Articulation doesn't accept usd_path in __init__
        # We need to add reference first, then wrap with Articulation
        from omni.isaac.core.utils.stage import add_reference_to_stage
        
        # NOTE: Ensure we add to a fresh prim path if it doesn't exist
        # Also need to make sure physics scene is initialized?
        # World() init should handle physics scene.
        
        add_reference_to_stage(usd_path=robot_url, prim_path="/World/Robot")
        
        # WAIT for stage to update?
        # self.world.step(render=False) # Maybe?
        
        # Register the articulation
        # IMPORTANT: "robot" name must be unique.
        # The crash 'NoneType' object has no attribute 'is_homogeneous' suggests physics parsing failed
        # likely because the prim at /World/Robot is not fully loaded or valid when Articulation is initialized.
        
        self.world.scene.add(
            Articulation(prim_path="/World/Robot", name="robot")
        )
        
        # Need to step once to ensure physics is parsed before reset?
        # Or maybe reset() is failing because it's called too early?
        # Let's add a step here.
        
        # self.world.reset() # This is where it crashes
        
        # Reset calls scene._finalize which calls articulation.initialize which checks physics
        # If we reset immediately after adding, sometimes physics isn't ready.
        # But World class usually handles this.
        
        # TRY: Explicitly initialize physics context?
        # or simply wait?
        
        # In Isaac Sim 4.0, maybe we need to ensure the Articulation is valid.
        # Let's try to reset AFTER getting the object, or splitting the reset.
        
        # Actually, self.world.reset() will initialize all objects in the scene.
        # The error suggests the physics view is None.
        
        # WORKAROUND: Force a timeline step before reset?
        # simulation_app.update()
        
        self.world.reset()
        
        self.robot = self.world.scene.get_object("robot")
        
        if ArticulationMotionController:
            self.controller = ArticulationMotionController(self.robot)
        else:
            print("⚠️ No MotionController available. IK commands will be ignored.")
            self.controller = None
        
        # Warm-up
        for _ in range(10):
            self.world.step(render=True)
        
        print("✅ Simulation initialized")

    def network_loop(self):
        """Background thread to handle TCP communication with teleop server"""
        last_state_time = 0
        last_frame_time = 0
        
        while self.running:
            # Capture video frame (10Hz)
            if self.running and time.time() - last_frame_time > 0.1:
                try:
                    data = self.rgb_annotator.get_data()
                    if data is not None:
                        # Data is (H, W, 4) or (H, W, 3) uint8
                        img = Image.fromarray(data[:, :, :3]) # Drop alpha if present
                        buf = io.BytesIO()
                        img.save(buf, format='JPEG', quality=60)
                        jpg_bytes = buf.getvalue()
                        
                        # Post to server
                        url = f"http://{self.host}:8000/api/v1/video/ingest"
                        requests.post(url, data=jpg_bytes, timeout=0.05)
                        last_frame_time = time.time()
                except Exception:
                    pass
            
            if self.sock is None:
                try:
                    print(f"🔌 Connecting to {self.host}:{self.port}...")
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.connect((self.host, self.port))
                    self.sock = s
                    print("✅ Connected to teleop server")
                except Exception as e:
                    print(f"❌ Connection failed: {e}. Retrying in 3s...")
                    time.sleep(3)
                    continue

            # Send state update (Pose)
            if self.robot and time.time() - last_state_time > 0.05: # 20Hz
                try:
                    # Get pose
                    pos, rot = self.robot.get_world_pose()
                    # Convert rot (w,x,y,z) to list
                    
                    state_msg = {
                        "type": "state",
                        "payload": {
                            "position": pos.tolist(),
                            "orientation": rot.tolist(), # w,x,y,z
                            "timestamp": time.time()
                        }
                    }
                    
                    data = json.dumps(state_msg) + "\n"
                    self.sock.sendall(data.encode("utf-8"))
                    last_state_time = time.time()
                except Exception as e:
                    print(f"⚠️ Error sending state: {e}")
                    # Don't break connection immediately on send error, might be transient
                    
            try:
                # Non-blocking receive would be better, but we used threading
                # We can set a timeout on the socket
                self.sock.settimeout(0.01)
                try:
                    data = self.sock.recv(4096)
                    if not data:
                        print("⚠️  Server disconnected")
                        self.sock.close()
                        self.sock = None
                        continue
                    
                    buffer = data.decode("utf-8")
                    for line in buffer.split("\n"):
                        if not line.strip():
                            continue
                        try:
                            msg = json.loads(line)
                            if msg.get("type") == "command":
                                payload = msg.get("payload", {})
                                if "target_position" in payload:
                                    self.target_pos = np.array(payload["target_position"], dtype=np.float32)
                                if "target_orientation" in payload:
                                    self.target_quat = np.array(payload["target_orientation"], dtype=np.float32)
                                if "gripper_state" in payload:
                                    self.gripper_state = float(payload["gripper_state"])
                        except json.JSONDecodeError:
                            pass
                except socket.timeout:
                    pass # No data
                    
            except Exception as e:
                print(f"❌ Network error: {e}")
                if self.sock:
                    self.sock.close()
                self.sock = None

    def run(self):
        self.init_sim()
        
        # Start network thread
        net_thread = threading.Thread(target=self.network_loop, daemon=True)
        net_thread.start()
        
        print("▶️  Starting simulation loop...")
        
        last_log_time = time.time()
        step_count = 0
        
        while simulation_app.is_running():
            self.world.step(render=True)
            step_count += 1
            
            # Apply IK to move robot
            if self.running and self.controller:
                action = self.controller.compute_inverse_kinematics(
                    target_end_effector_position=self.target_pos,
                    target_end_effector_orientation=self.target_quat,
                    end_effector_frame_name=self.ee_frame
                )
                self.robot.apply_action(action)
                
                # Apply gripper state (Simple naive implementation)
                # Assumes gripper joints are not controlled by IK
                if self.gripper_state >= 0:
                    # Find gripper joints (heuristic)
                    # For now, just print if state changes to avoid spam
                    # TODO: Implement actual gripper control based on robot specific joint names
                    pass
            
            # Periodic logging
            if time.time() - last_log_time > 10.0:
                if self.robot:
                    pos, _ = self.robot.get_world_pose()
                    print(f"📊 [Step {step_count}] Robot pos: {pos}, Target: {self.target_pos}")
                last_log_time = time.time()
        
        print("🛑 Shutting down...")
        self.running = False
        simulation_app.close()

if __name__ == "__main__":
    client = IsaacSimTCPClient(args.host, args.port, args.robot_usd, args.ee_frame)
    client.run()
