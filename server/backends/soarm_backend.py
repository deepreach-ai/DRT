"""
SO-ARM Backend using LeRobot SO-101 Follower
"""
from typing import Tuple, Optional, Dict, Any
import numpy as np
import time
import threading
import sys
import os

# Add lerobot to path if needed
lerobot_path = os.path.expanduser("~/lerobot/src")
if lerobot_path not in sys.path:
    sys.path.insert(0, lerobot_path)

from robot_backend import RobotBackend, BackendStatus

# Import OpenCV
try:
    import cv2
    OPENCV_AVAILABLE = True
except ImportError:
    cv2 = None
    OPENCV_AVAILABLE = False

# Import RealSense
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    rs = None
    REALSENSE_AVAILABLE = False

# Import LeRobot SO-101
try:
    from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
    from lerobot.robots.robot import Robot
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.cameras.utils import make_cameras_from_configs
    LEROBOT_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Could not import LeRobot: {e}")
    LEROBOT_AVAILABLE = False


class SafeFeetechMotorsBus(FeetechMotorsBus):
    """
    A safer version of FeetechMotorsBus that handles multi-turn positions 
    gracefully during calibration.
    """
    def _get_half_turn_homings(self, positions: Dict[Any, Any]) -> Dict[Any, Any]:
        """
        Override to handle multi-turn positions by using modulo.
        Original implementation fails if position is outside single-turn range 
        because calculated offset exceeds 12-bit signed limit.
        """
        half_turn_homings = {}
        for motor, pos in positions.items():
            model = self._get_motor_model(motor)
            max_res = self.model_resolution_table[model] - 1
            resolution = max_res + 1
            
            # Apply modulo to handle multi-turn (e.g. 32459 -> 3787)
            pos_mod = pos % resolution
            
            # Calculate offset to center the range
            # We want: (pos_mod - offset) approx equals (max_res / 2)
            target_center = int(max_res / 2)
            offset = pos_mod - target_center
            
            # Ensure offset is within valid range [-2047, 2047] for 12-bit signed magnitude
            # (STS3215 Homing Offset is limited to this range)
            max_offset = 2047
            if offset > max_offset:
                offset = max_offset
            elif offset < -max_offset:
                offset = -max_offset
                
            half_turn_homings[motor] = offset
            
        return half_turn_homings


class FlexibleSO101Follower(SO101Follower):
    """
    A subclass of SO101Follower that dynamically detects available motors
    and initializes with only the connected ones.
    """
    def __init__(self, config: SO101FollowerConfig):
        # Initialize Robot base (skipping SO101Follower.__init__ to avoid strict check)
        Robot.__init__(self, config)
        self.config = config
        
        # Define all expected motors (copied from SO101Follower)
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        all_motors = {
            "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
            "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
            "elbow_flex": Motor(3, "sts3215", norm_mode_body),
            "wrist_flex": Motor(4, "sts3215", norm_mode_body),
            "wrist_roll": Motor(5, "sts3215", norm_mode_body),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        }
        
        print(f"[FlexibleSO101Follower] Scanning for motors on {config.port}...")
        found_motors = {}
        
        try:
            # Create a temporary bus to scan
            # We pass empty motors dict to avoid immediate validation
            temp_bus = SafeFeetechMotorsBus(port=config.port, motors={})
            temp_bus.connect(handshake=False)
            
            # Ping each expected ID
            for name, motor in all_motors.items():
                try:
                    # Ping returns model number if found, None otherwise
                    model = temp_bus.ping(motor.id)
                    if model is not None:
                        found_motors[name] = motor
                        # print(f"  - Found {name} (ID {motor.id})")
                    else:
                        print(f"  - Missing {name} (ID {motor.id})")
                except Exception:
                    print(f"  - Error checking {name} (ID {motor.id})")
            
            temp_bus.disconnect()
            
        except Exception as e:
            print(f"[FlexibleSO101Follower] Scan failed: {e}")
            # If scan fails completely, fall back to full list (will likely fail again in main init)
            found_motors = all_motors

        if not found_motors:
            print("[FlexibleSO101Follower] No motors found! Trying full list.")
            found_motors = all_motors
        else:
            print(f"[FlexibleSO101Follower] Initializing with {len(found_motors)}/{len(all_motors)} motors")

        # Removed: forcing gripper into found_motors
        # if "gripper" not in found_motors:
        #    found_motors["gripper"] = all_motors["gripper"]
        #    print("[FlexibleSO101Follower] Added gripper explicitly")

        # Filter calibration to match found motors
        if self.calibration:
            filtered_calibration = {k: v for k, v in self.calibration.items() if k in found_motors}
            if len(filtered_calibration) != len(self.calibration):
                print(f"[FlexibleSO101Follower] Filtered calibration from {len(self.calibration)} to {len(filtered_calibration)} motors")
            self.calibration = filtered_calibration

        # Initialize the real bus with found motors
        self.bus = SafeFeetechMotorsBus(
            port=self.config.port,
            motors=found_motors,
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)


class SOARMKinematics:
    """
    Simple Kinematics for SO-ARM100 (5-DOF)
    """
    # Link lengths in meters
    L_BASE = 0.10   # Base to Shoulder axis
    L_UPPER = 0.12  # Shoulder to Elbow
    L_FORE = 0.12   # Elbow to Wrist
    L_HAND = 0.10   # Wrist to End Effector

    @classmethod
    def forward_kinematics(cls, joints: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Compute FK for 5-DOF arm (joints in Radians)"""
        if len(joints) < 5:
            return np.zeros(3), np.array([1, 0, 0, 0])
            
        t1, t2, t3, t4, t5 = joints[:5]
        
        sin = np.sin
        cos = np.cos
        
        # Global angles
        g2 = t2
        g3 = t2 + t3
        g4 = t2 + t3 + t4
        
        # Radial distance (in X-Y plane)
        r = (cls.L_UPPER * sin(g2) + 
             cls.L_FORE * sin(g3) + 
             cls.L_HAND * sin(g4))
             
        # Z height (relative to shoulder)
        z_shoulder = (cls.L_UPPER * cos(g2) + 
                      cls.L_FORE * cos(g3) + 
                      cls.L_HAND * cos(g4))
                      
        z = cls.L_BASE + z_shoulder
        
        # X, Y
        x = r * cos(t1)
        y = r * sin(t1)
        
        position = np.array([x, y, z])
        
        # Orientation (Yaw, Pitch, Roll) -> Quaternion
        cy = cos(t1 * 0.5)
        sy = sin(t1 * 0.5)
        cp = cos(g4 * 0.5)
        sp = sin(g4 * 0.5)
        cr = cos(t5 * 0.5)
        sr = sin(t5 * 0.5)

        w = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        orientation = np.array([w, qx, qy, qz])
        return position, orientation

    @classmethod
    def inverse_kinematics(cls, target_pos: np.ndarray, target_pitch: float = 0.0, target_roll: float = 0.0) -> Optional[np.ndarray]:
        """Simple Analytical IK for 5-DOF (returns Radians)"""
        x, y, z = target_pos
        
        # 1. Joint 1: Pan
        t1 = np.arctan2(y, x)
        
        # 2. Solve Planar IK
        r_tip = np.sqrt(x**2 + y**2)
        z_tip = z - cls.L_BASE
        
        r_wrist = r_tip - cls.L_HAND * np.sin(target_pitch)
        z_wrist = z_tip - cls.L_HAND * np.cos(target_pitch)
        
        d_sq = r_wrist**2 + z_wrist**2
        d = np.sqrt(d_sq)
        
        cos_t3 = (d_sq - cls.L_UPPER**2 - cls.L_FORE**2) / (2 * cls.L_UPPER * cls.L_FORE)
        
        if abs(cos_t3) > 1.0: return None
            
        t3 = np.arccos(cos_t3) # Range [0, pi]
        
        alpha = np.arctan2(r_wrist, z_wrist)
        cos_beta = (cls.L_UPPER**2 + d_sq - cls.L_FORE**2) / (2 * cls.L_UPPER * d)
        if abs(cos_beta) > 1.0: return None
        beta = np.arccos(cos_beta)
        
        t2 = alpha - beta
        t4 = target_pitch - t2 - t3
        
        return np.array([t1, t2, t3, t4, target_roll])



class SOARMBackend(RobotBackend):
    """Backend for SO-ARM100/SO-101 robot using LeRobot Standard API"""
    
    def __init__(self, port: str = "/dev/tty.usbmodem5B3E1224691", name: str = "so_arm"):
        super().__init__(name)
        # Allow overriding port via env var if not passed explicitly?
        # Actually BackendFactory passes it from config or env.
        # But if we use default arg, we might be stuck.
        
        # If passed port is default and env is set, use env?
        # Better: let BackendFactory handle it.
        # But here we see the backend is instantiated with port="/dev/tty.usbmodem5B3E1187881" from somewhere.
        
        self.port = port
        self.robot = None
        self.current_position = np.array([0.0, 0.0, 0.0])  # Will be updated from robot
        self.current_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # quat [w,x,y,z]
        self.command_count = 0
        
        # Camera state
        self.cameras = {}
        self.camera_started = False
        
        if not LEROBOT_AVAILABLE:
            raise RuntimeError("LeRobot is not available. Please install it first.")
    
    def connect(self) -> bool:
        """Connect to SO-ARM robot and Camera"""
        try:
            print(f"[SOARMBackend] Connecting to SO-ARM at {self.port}...")
            self.status = BackendStatus.CONNECTING
            
            # 1. Connect to Robot using Standard LeRobot Config
            config = SO101FollowerConfig(
                port=self.port,
                use_degrees=True, # IMPORTANT: Use degrees for easier mapping
            )
            
            # Initialize robot
            self.robot = FlexibleSO101Follower(config)
            
            # Pass calibrate=False to avoid interactive prompts
            # If calibration is missing, this might be risky, but LeRobot usually has defaults
            self.robot.connect(calibrate=False)
            
            print(f"[SOARMBackend] Enabling motor torque...")
            self.robot.bus.enable_torque()
            print(f"[SOARMBackend] ✓ Connected to SO-ARM")

            # 2. Connect to Cameras
            enable_cameras = os.getenv("TELEOP_ENABLE_CAMERAS", "true").lower() == "true"
            if enable_cameras:
                self._init_cameras()
            
            self.status = BackendStatus.CONNECTED
            self.last_update_time = time.time()
            return True
            
        except Exception as e:
            print(f"[SOARMBackend] ✗ Connection failed: {e}")
            import traceback
            traceback.print_exc()
            self.status = BackendStatus.ERROR
            return False

    def _init_cameras(self):
        """Initialize external cameras"""
        if not OPENCV_AVAILABLE:
            return
            
        print("[SOARMBackend] Scanning for cameras...")
        self.cameras = {}
        roles = ["side_1", "side_2", "top"]
        found_count = 0
        
        for i in range(1, 10):
            if found_count >= len(roles): break
            try:
                cap = cv2.VideoCapture(i)
                if cap.isOpened():
                    ret, _ = cap.read()
                    if ret:
                        role = roles[found_count]
                        self.cameras[role] = {'type': 'opencv', 'obj': cap}
                        print(f"[SOARMBackend] ✓ Found {role} at ID {i}")
                        found_count += 1
                    else:
                        cap.release()
                else:
                    cap.release()
            except:
                pass
        
        if self.cameras:
            self.camera_started = True

    def disconnect(self):
        """Disconnect"""
        try:
            # Stop Cameras
            for cam in self.cameras.values():
                if cam['type'] == 'opencv':
                    cam['obj'].release()
            self.cameras = {}
            self.camera_started = False

            # Disconnect Robot
            if self.robot is not None:
                self.robot.disconnect()
                self.robot = None
                print("[SOARMBackend] Robot disconnected")
            
            self.status = BackendStatus.DISCONNECTED
        except Exception as e:
            print(f"[SOARMBackend] Error during disconnect: {e}")
            self.status = BackendStatus.ERROR

    def send_target_pose(self, position: np.ndarray, 
                         orientation: np.ndarray,
                         velocity_limit: float = 0.1,
                         gripper_state: float = -1.0,
                         handedness: str = "right") -> bool:
        """
        Send target pose to SO-ARM using Analytical IK (Degrees)
        """
        if not self.is_connected() or self.robot is None:
            return False
        
        try:
            with self.update_lock:
                # 1. Determine Target Pitch and Roll from Orientation Quaternion
                # Convert Quat (w, x, y, z) to Euler ZYX
                w, x, y, z = orientation
                
                # Yaw (Z-axis rotation)
                # siny_cosp = 2 * (w * z + x * y)
                # cosy_cosp = 1 - 2 * (y * y + z * z)
                # yaw = np.arctan2(siny_cosp, cosy_cosp)
                
                # Pitch (Y-axis rotation)
                sinp = 2 * (w * y - z * x)
                if abs(sinp) >= 1:
                    pitch = np.copysign(np.pi / 2, sinp)
                else:
                    pitch = np.arcsin(sinp)
                
                # Roll (X-axis rotation)
                sinr_cosp = 2 * (w * x + y * z)
                cosr_cosp = 1 - 2 * (x * x + y * y)
                roll = np.arctan2(sinr_cosp, cosr_cosp)
                
                target_pitch = pitch
                target_roll = roll

                # 2. Inverse Kinematics (Returns Radians)
                ik_joints_rad = SOARMKinematics.inverse_kinematics(position, target_pitch, target_roll)
                
                if ik_joints_rad is None:
                    # If unreachable, do not return False immediately, 
                    # as this will stop the controller loop if treated as fatal error.
                    # Instead, just don't move and log.
                    # print(f"[SOARMBackend] IK Unreachable: {position}")
                    return True # Pretend success to keep loop alive
                
                # 3. Convert Radians to Degrees
                # LeRobot (use_degrees=True) expects degrees
                joints_deg = np.degrees(ik_joints_rad)
                
                # 4. Handle Gripper
                g_val = 0.0
                if gripper_state >= 0:
                    g_val = gripper_state * 100.0 # 0.0-1.0 -> 0-100
                
                # 5. Smart Wrapping for Multi-Turn Safety
                # Prevent spinning if motor is at e.g. -1200 degrees
                current_obs = self.robot.get_observation()
                
                # Map names
                names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
                
                final_command = {}
                
                for i, name in enumerate(names):
                    target_angle = joints_deg[i]
                    current_angle = 0.0
                    
                    # Get current angle
                    if current_obs:
                        val = current_obs.get(name) or current_obs.get(f"{name}.pos")
                        if val is not None:
                            current_angle = float(val)
                    
                    # Debug: Check if we are reading zeros
                    # if i == 0 and abs(current_angle) < 0.1 and abs(target_angle) > 10:
                    #     print(f"[SOARMBackend] Warning: {name} read 0.0, target {target_angle:.1f}")
                    
                    # Find closest equivalent angle
                    # We want to minimize |(target_angle + k*360) - current_angle|
                    
                    # 1. Calculate diff in [-180, 180]
                    diff = (target_angle - current_angle + 180) % 360 - 180
                    
                    # 2. New target is current + diff
                    effective_target = current_angle + diff
                    
                    # --- ADDED: Safety Check for Large Jumps ---
                    # If effective_target differs significantly from current_angle (> 45 degrees),
                    # clamp it to prevent violent movement.
                    max_step = 45.0 # Increased from 10.0 to 45.0 to avoid "tiny move" stuck issue
                    step = effective_target - current_angle
                    
                    # Special handling for wrist_roll large jumps (likely IK flipping)
                    # If step is ~180, it's definitely a flip.
                    if name == "wrist_roll" and abs(step) > 90:
                         print(f"[SOARMBackend] ⚠️ Suppressing wrist_roll flip: {step:.1f} deg. Holding position.")
                         # If it tries to flip 180, just hold current position
                         effective_target = current_angle
                         step = 0.0
                    
                    if abs(step) > max_step:
                        # Log occasionally
                        if self.command_count % 20 == 0:
                             print(f"[SOARMBackend] ⚠️ Clamping fast move: {name} current={current_angle:.1f} target={effective_target:.1f} step={step:.1f}")
                        step = np.clip(step, -max_step, max_step)
                        effective_target = current_angle + step
                    
                    final_command[name] = effective_target
                    
                # Add gripper (ONLY IF EXISTS)
                if "gripper" in self.robot.bus.motors:
                    final_command["gripper"] = g_val
                
                # 6. Send Command
                # Debug log for command
                # print(f"[SOARMBackend] Moving to: {final_command}")
                
                # Check for NaNs
                for k, v in final_command.items():
                    if not np.isfinite(v):
                        print(f"[SOARMBackend] ⚠️ NaN in command for {k}! Ignoring.")
                        return True
                
                self.robot.bus.sync_write("Goal_Position", final_command)
                
                self.current_position = position
                self.current_orientation = orientation
                self.command_count += 1
                self.last_update_time = time.time()
                return True

        except Exception as e:
            print(f"[SOARMBackend] Error sending command: {e}")
            return False

    def send_joint_positions(self, joint_positions: np.ndarray) -> bool:
        """
        Direct joint control (Array of degrees + gripper 0-100)
        """
        if not self.is_connected() or self.robot is None:
            return False
        try:
            # Assume order: Pan, Lift, Elbow, W_Flex, W_Roll, Gripper
            names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
            
            command = {}
            for i, name in enumerate(names):
                if i < len(joint_positions):
                    # Only add if motor exists on bus
                    if name in self.robot.bus.motors:
                        command[name] = float(joint_positions[i])
            
            if command:
                self.robot.bus.sync_write("Goal_Position", command)
            return True
        except Exception as e:
            print(f"Error sending joints: {e}")
            return False

    def get_current_pose(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get current robot pose using Forward Kinematics"""
        if not self.is_connected() or self.robot is None:
            return None, None
        
        try:
            with self.update_lock:
                # Read observation (Degrees)
                # robot.get_observation() returns dictionary of positions
                # e.g. {'shoulder_pan.pos': 10.5, ...}
                # But SO101Follower might just return {'shoulder_pan': ...} depending on version
                # My test showed keys like 'shoulder_pan.pos'
                
                obs = self.robot.get_observation()
                if not obs:
                    return None, None
                    
                # Extract joint values in order
                # Note: keys have suffix '.pos' usually?
                # My verify script output: ['shoulder_pan.pos', ...]
                
                names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
                joints_deg = []
                for name in names:
                    # Try name or name.pos
                    val = obs.get(name)
                    if val is None:
                        val = obs.get(f"{name}.pos")
                    if val is None:
                        # Fallback or error
                        return self.current_position, self.current_orientation
                    joints_deg.append(val)
                
                joints_deg = np.array(joints_deg)
                
                # Convert Degrees -> Radians
                joints_rad = np.radians(joints_deg)
                
                # Compute FK
                pos, rot = SOARMKinematics.forward_kinematics(joints_rad)
                self.current_position = pos
                self.current_orientation = rot
                
                return self.current_position.copy(), self.current_orientation.copy()
                
        except Exception as e:
            print(f"[SOARMBackend] Error reading pose: {e}")
            return None, None
    
    def get_current_joints(self) -> Optional[np.ndarray]:
        # Helper for status
        if not self.is_connected(): return None
        obs = self.robot.get_observation()
        if not obs: return None
        # Extract values
        return np.array([obs.get(k, 0.0) for k in sorted(obs.keys())])
        
    def render(self, width: int = 960, height: int = 540) -> Optional[bytes]:
        # Same render logic as before
        if not self.camera_started or not self.cameras:
            return None
        try:
            frames = {}
            for name, cam in self.cameras.items():
                if cam['type'] == 'opencv':
                    ret, frame = cam['obj'].read()
                    if ret: frames[name] = frame
            
            if not frames: return None
            
            # Simple stitching
            canvas = np.zeros((height, width, 3), dtype=np.uint8)
            slot_width = width // 3
            
            def place(img, idx):
                if img is None: return
                h, w = img.shape[:2]
                scale = slot_width / w
                new_h = int(h * scale)
                resized = cv2.resize(img, (slot_width, new_h))
                y = (height - new_h) // 2
                x = idx * slot_width
                if y < 0:
                    resized = resized[-y:-y+height, :]
                    y = 0
                canvas[y:y+resized.shape[0], x:x+slot_width] = resized

            place(frames.get('side_1'), 0)
            place(frames.get('top'), 1)
            place(frames.get('side_2'), 2)
            
            _, jpeg = cv2.imencode('.jpg', canvas)
            return jpeg.tobytes()
        except:
            return None

    def get_status(self) -> Dict[str, Any]:
        """Get SO-ARM backend status"""
        status = {
            'name': self.name,
            'status': self.status.value,
            'port': self.port,
            'command_count': self.command_count,
            'last_update': self.last_update_time,
            'lerobot_available': LEROBOT_AVAILABLE,
            'camera_available': REALSENSE_AVAILABLE,
            'camera_started': self.camera_started
        }
        return status
